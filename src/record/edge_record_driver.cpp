#include "edge_record_driver.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "../drivers/mzio.h"
#include "../drivers/write_edge_monitor.h"
#include "../streams/wav_sample_stream.h"

#if !defined(TIMER5_OVF_vect)
#error "EDGE record driver requires Timer5 on ATmega2560"
#endif

#define EDGE_FIFO_BYTES WAV_SAMPLE_STREAM_BUFFER_BYTES
#define EDGE_FIFO_CAPACITY (EDGE_FIFO_BYTES - 1U)
#define EDGE_FIFO_MASK (EDGE_FIFO_BYTES - 1U)

static volatile uint16_t edge_read_sequence = 0U;
static volatile uint16_t edge_write_sequence = 0U;
static volatile uint8_t edge_state = EDGE_RECORD_DRIVER_STOPPED;

/*
   Timer5 is deliberately configured per output format:
   - L16: F_CPU/64 = 250 kHz, 4 us/tick, exactly four ticks per L16 unit.
   - LEP: F_CPU/8  = 2 MHz, 0.5 us/tick, exactly 100 ticks per LEP unit.

   The overflow count is only eight bits. This still covers more than 5 s:
   - L16: 255 * 262.144 ms = 66.8 s
   - LEP: 255 *  32.768 ms = 8.36 s
*/
static volatile uint8_t timer5_overflow_count = 0U;
static volatile uint16_t paused_counter = 0U;
static volatile uint8_t paused_overflows = 0U;
static uint16_t unit_ticks = 100U;
static uint8_t timer5_clock_bits = _BV(CS51);
static volatile uint8_t active_level = 0U;
static volatile uint8_t initial_level = 0U;

/* Error-diffusion residual, bounded to +/- half of one output unit. */
static int8_t quantization_residual_ticks = 0;

/* One unpaired 1..15-unit slot is held until the next short slot arrives. */
static volatile uint8_t pending_short_units = 0U;

static inline bool edge_fifo_reserve_from_isr(uint8_t count)
{
    uint16_t used = (uint16_t)(edge_write_sequence - edge_read_sequence);
    return ((uint16_t)(EDGE_FIFO_CAPACITY - used) >= count);
}

static inline bool edge_push_byte_from_isr(uint8_t value)
{
    uint16_t w = edge_write_sequence;

    if (!edge_fifo_reserve_from_isr(1U))
    {
        return false;
    }

    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = value;
    asm volatile("" ::: "memory");
    edge_write_sequence = (uint16_t)(w + 1U);
    return true;
}

static inline bool edge_flush_pending_short_from_isr(void)
{
    uint8_t pending = pending_short_units;

    if (pending == 0U)
    {
        return true;
    }
    if (!edge_push_byte_from_isr((uint8_t)(pending << 4)))
    {
        return false;
    }
    pending_short_units = 0U;
    return true;
}

static inline bool edge_push_short_units_from_isr(uint8_t units)
{
    uint8_t pending = pending_short_units;

    if ((units == 0U) || (units > 15U))
    {
        return false;
    }
    if (pending == 0U)
    {
        pending_short_units = units;
        return true;
    }
    if (!edge_push_byte_from_isr((uint8_t)((pending << 4) | units)))
    {
        return false;
    }
    pending_short_units = 0U;
    return true;
}

static inline bool edge_push_unit_from_isr(uint8_t units)
{
    uint16_t w;

    if ((units < 16U) || (units > 127U))
    {
        return false;
    }
    if (!edge_flush_pending_short_from_isr() || !edge_fifo_reserve_from_isr(2U))
    {
        return false;
    }

    w = edge_write_sequence;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = EDGE_RECORD_TOKEN_UNIT;
    w++;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = units;
    w++;
    asm volatile("" ::: "memory");
    edge_write_sequence = w;
    return true;
}

static inline bool edge_push_long_units_from_isr(uint32_t units)
{
    uint16_t w;

    if (units <= 127UL)
    {
        return false;
    }
    if (!edge_flush_pending_short_from_isr() ||
        !edge_fifo_reserve_from_isr(EDGE_RECORD_TOKEN_LONG_BYTES))
    {
        return false;
    }

    w = edge_write_sequence;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = EDGE_RECORD_TOKEN_LONG;
    w++;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = (uint8_t)(units & 0xFFUL);
    w++;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] =
        (uint8_t)((units >> 8) & 0xFFUL);
    w++;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] =
        (uint8_t)((units >> 16) & 0xFFUL);
    w++;
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] =
        (uint8_t)((units >> 24) & 0xFFUL);
    w++;
    asm volatile("" ::: "memory");
    edge_write_sequence = w;
    return true;
}

/*
   Fast path used by normal cassette/5 kHz edges. It has no 32-bit arithmetic.
   L16 is shift/mask only. LEP uses repeated subtraction; at 5 kHz it executes
   exactly two iterations for a 100 us half-wave.
*/
static inline bool edge_quantize_no_overflow_from_isr(uint16_t ticks,
                                                       uint8_t *units_out)
{
    int16_t adjusted;
    uint16_t units = 0U;

    if (units_out == NULL)
    {
        return false;
    }

    /* Do not enter the 16-bit fast path for a value that may overflow int16. */
    if ((unit_ticks == 100U && ticks > 12850U) ||
        (unit_ticks == 4U && ticks > 520U))
    {
        return false;
    }

    adjusted = (int16_t)ticks + (int16_t)quantization_residual_ticks;
    if (adjusted < 1)
    {
        adjusted = 1;
    }

    if (unit_ticks == 4U)
    {
        units = (uint16_t)((adjusted + 2) >> 2);
    }
    else
    {
        uint16_t rounded = (uint16_t)(adjusted + 50);
        while (rounded >= 100U)
        {
            rounded = (uint16_t)(rounded - 100U);
            units++;
            if (units > 127U)
            {
                return false;
            }
        }
    }

    if ((units == 0U) || (units > 127U))
    {
        return false;
    }

    quantization_residual_ticks =
        (int8_t)(adjusted - (int16_t)(units * unit_ticks));
    *units_out = (uint8_t)units;
    return true;
}

/* Rare slow path for pulses longer than one normal Timer5 cycle. */
static inline bool edge_emit_long_elapsed_from_isr(uint8_t overflows,
                                                    uint16_t ticks)
{
    uint32_t elapsed_ticks = ((uint32_t)overflows << 16) | (uint32_t)ticks;
    int32_t adjusted = (int32_t)elapsed_ticks +
                       (int32_t)quantization_residual_ticks;
    uint32_t units;
    int32_t residual;

    if (adjusted < 1L)
    {
        adjusted = 1L;
    }

    units = (uint32_t)((adjusted + (int32_t)(unit_ticks / 2U)) /
                       (int32_t)unit_ticks);
    if (units == 0UL)
    {
        units = 1UL;
    }
    residual = adjusted - (int32_t)(units * (uint32_t)unit_ticks);
    quantization_residual_ticks = (int8_t)residual;

    if (units <= 15UL)
    {
        return edge_push_short_units_from_isr((uint8_t)units);
    }
    if (units <= 127UL)
    {
        return edge_push_unit_from_isr((uint8_t)units);
    }
    return edge_push_long_units_from_isr(units);
}

static inline bool edge_emit_elapsed_from_isr(uint8_t overflows,
                                              uint16_t ticks)
{
    uint8_t units;

    if ((overflows == 0U) &&
        edge_quantize_no_overflow_from_isr(ticks, &units))
    {
        if (units <= 15U)
        {
            return edge_push_short_units_from_isr(units);
        }
        return edge_push_unit_from_isr(units);
    }

    return edge_emit_long_elapsed_from_isr(overflows, ticks);
}

/*
   Snapshot elapsed time since the last accepted edge, then start the next
   interval from zero. The TOV5 race is handled in the conventional way: when
   the flag is pending and TCNT5 is in its low half, one overflow has occurred
   but its ISR has not incremented timer5_overflow_count yet.
*/
static inline void edge_take_elapsed_and_restart_from_isr(uint8_t *overflows,
                                                           uint16_t *ticks)
{
    uint8_t count = timer5_overflow_count;
    uint16_t value = TCNT5;

    if (((TIFR5 & _BV(TOV5)) != 0U) && (value < 0x8000U))
    {
        count++;
    }

    timer5_overflow_count = 0U;
    TCNT5 = 0U;
    TIFR5 = _BV(TOV5);

    *overflows = count;
    *ticks = value;
}

static inline void edge_stop_pcint_from_isr(void)
{
    write_edge_monitor_stop();
}

static inline void edge_stop_timer_from_isr(void)
{
    edge_stop_pcint_from_isr();
    TIMSK5 &= (uint8_t)~_BV(TOIE5);
    TCCR5B = 0U;
    TCCR5A = 0U;
    TIFR5 = _BV(TOV5);
}

static inline void edge_start_timer_from_isr(uint8_t overflows, uint16_t ticks)
{
    TCCR5A = 0U;
    TCCR5B = 0U;
    TCNT5 = ticks;
    timer5_overflow_count = overflows;
    TIFR5 = _BV(TOV5);
    TIMSK5 |= _BV(TOIE5);
    TCCR5B = timer5_clock_bits;
}

static inline void edge_enable_capture_from_isr(void)
{
    write_edge_monitor_begin_edge_capture();
}

void edge_record_driver_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        edge_stop_timer_from_isr();
        edge_read_sequence = 0U;
        edge_write_sequence = 0U;
        timer5_overflow_count = 0U;
        paused_counter = 0U;
        paused_overflows = 0U;
        unit_ticks = 100U;
        timer5_clock_bits = _BV(CS51);
        active_level = 0U;
        initial_level = 0U;
        pending_short_units = 0U;
        quantization_residual_ticks = 0;
        edge_state = EDGE_RECORD_DRIVER_STOPPED;
    }
}

bool edge_record_driver_prepare(uint8_t unit_us)
{
    edge_record_driver_init();

    if (unit_us == 16U)
    {
        /* F_CPU/64 = 250 kHz, exactly four ticks per 16 us. */
        unit_ticks = 4U;
        timer5_clock_bits = (uint8_t)(_BV(CS51) | _BV(CS50));
    }
    else if (unit_us == 50U)
    {
        /* F_CPU/8 = 2 MHz, exactly 100 ticks per 50 us. */
        unit_ticks = 100U;
        timer5_clock_bits = _BV(CS51);
    }
    else
    {
        edge_state = EDGE_RECORD_DRIVER_BAD_ARGUMENT;
        return false;
    }

    edge_state = EDGE_RECORD_DRIVER_READY;
    return true;
}

bool edge_record_driver_start(void)
{
    if (edge_state != EDGE_RECORD_DRIVER_READY)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        DDRJ &= (uint8_t)~_BV(PJ0);
        PORTJ &= (uint8_t)~_BV(PJ0);

        edge_start_timer_from_isr(0U, 0U);
        active_level = mz_write_sample_from_isr();
        initial_level = active_level;
        paused_counter = 0U;
        paused_overflows = 0U;
        pending_short_units = 0U;
        quantization_residual_ticks = 0;
        edge_state = EDGE_RECORD_DRIVER_RUNNING;
        edge_enable_capture_from_isr();
    }
    return true;
}

bool edge_record_driver_pause(void)
{
    uint8_t overflows;
    uint16_t ticks;

    if (edge_state != EDGE_RECORD_DRIVER_RUNNING)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        edge_stop_pcint_from_isr();
        edge_take_elapsed_and_restart_from_isr(&overflows, &ticks);
        TCCR5B = 0U;
        paused_overflows = overflows;
        paused_counter = ticks;
        edge_state = EDGE_RECORD_DRIVER_PAUSED;
    }
    return true;
}

bool edge_record_driver_resume(void)
{
    uint8_t level;
    bool ok = true;

    if (edge_state != EDGE_RECORD_DRIVER_PAUSED)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        level = mz_write_sample_from_isr();

        if (level != active_level)
        {
            /* Close the old level at pause, start the new level at resume. */
            if (!edge_emit_elapsed_from_isr(paused_overflows, paused_counter))
            {
                edge_stop_timer_from_isr();
                edge_state = EDGE_RECORD_DRIVER_OVERRUN;
                ok = false;
            }
            else
            {
                active_level = level;
                edge_start_timer_from_isr(0U, 0U);
            }
        }
        else
        {
            /* Continue the same pulse; paused wall time is not counted. */
            edge_start_timer_from_isr(paused_overflows, paused_counter);
        }

        if (ok)
        {
            edge_state = EDGE_RECORD_DRIVER_RUNNING;
            edge_enable_capture_from_isr();
        }
    }
    return ok;
}

void edge_record_driver_stop(void)
{
    uint8_t overflows = 0U;
    uint16_t ticks = 0U;
    bool valid;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        valid = ((edge_state == EDGE_RECORD_DRIVER_RUNNING) ||
                 (edge_state == EDGE_RECORD_DRIVER_PAUSED));
        if (!valid)
        {
            return;
        }

        edge_stop_pcint_from_isr();
        if (edge_state == EDGE_RECORD_DRIVER_RUNNING)
        {
            edge_take_elapsed_and_restart_from_isr(&overflows, &ticks);
        }
        else
        {
            overflows = paused_overflows;
            ticks = paused_counter;
        }

        TCCR5B = 0U;
        TIMSK5 &= (uint8_t)~_BV(TOIE5);
        TIFR5 = _BV(TOV5);

        if (!edge_emit_elapsed_from_isr(overflows, ticks) ||
            !edge_flush_pending_short_from_isr())
        {
            edge_state = EDGE_RECORD_DRIVER_OVERRUN;
        }
        else
        {
            edge_state = EDGE_RECORD_DRIVER_STOPPED;
        }
    }
}

void edge_record_driver_abort(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        edge_stop_timer_from_isr();
        pending_short_units = 0U;
        edge_state = EDGE_RECORD_DRIVER_STOPPED;
    }
}

edge_record_driver_state_t edge_record_driver_get_state(void)
{
    uint8_t state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        state = edge_state;
    }
    return (edge_record_driver_state_t)state;
}

uint16_t edge_record_driver_available_bytes(void)
{
    uint16_t r;
    uint16_t w;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        r = edge_read_sequence;
        w = edge_write_sequence;
    }
    return (uint16_t)(w - r);
}

bool edge_record_driver_pop_byte(uint8_t *value)
{
    bool result = false;

    if (value == NULL)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint16_t r = edge_read_sequence;
        if (r != edge_write_sequence)
        {
            *value = wav_sample_stream_isr_bytes[r & EDGE_FIFO_MASK];
            edge_read_sequence = (uint16_t)(r + 1U);
            result = true;
        }
    }
    return result;
}

uint8_t edge_record_driver_fill_percent(void)
{
    uint32_t percent = ((uint32_t)edge_record_driver_available_bytes() * 100UL) /
                       EDGE_FIFO_CAPACITY;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

uint32_t edge_record_driver_get_captured_active_ticks(void)
{
    uint8_t overflows;
    uint16_t ticks;
    edge_record_driver_state_t state = edge_record_driver_get_state();

    if ((state != EDGE_RECORD_DRIVER_RUNNING) &&
        (state != EDGE_RECORD_DRIVER_PAUSED))
    {
        return 0UL;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (state == EDGE_RECORD_DRIVER_PAUSED)
        {
            overflows = paused_overflows;
            ticks = paused_counter;
        }
        else
        {
            overflows = timer5_overflow_count;
            ticks = TCNT5;
            if (((TIFR5 & _BV(TOV5)) != 0U) && (ticks < 0x8000U))
            {
                overflows++;
            }
        }
    }
    return ((uint32_t)overflows << 16) | (uint32_t)ticks;
}

uint8_t edge_record_driver_get_initial_level(void)
{
    uint8_t result;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = initial_level;
    }
    return result;
}

ISR(TIMER5_OVF_vect)
{
    if (edge_state != EDGE_RECORD_DRIVER_RUNNING)
    {
        return;
    }

    if (timer5_overflow_count == 0xFFU)
    {
        edge_stop_timer_from_isr();
        edge_state = EDGE_RECORD_DRIVER_TOO_LONG;
        return;
    }
    timer5_overflow_count++;
}

void edge_record_driver_on_write_edge_from_isr(uint8_t new_level)
{
    uint8_t overflows;
    uint16_t ticks;

    if (edge_state != EDGE_RECORD_DRIVER_RUNNING)
    {
        return;
    }
    if (new_level == active_level)
    {
        return;
    }

    edge_take_elapsed_and_restart_from_isr(&overflows, &ticks);
    if (!edge_emit_elapsed_from_isr(overflows, ticks))
    {
        edge_stop_timer_from_isr();
        edge_state = EDGE_RECORD_DRIVER_OVERRUN;
        return;
    }

    active_level = new_level;
}
