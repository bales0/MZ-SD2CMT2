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

/* Timer5 runs at F_CPU/8 = 2 MHz => 0.5us resolution. */
static volatile uint16_t timer5_high_word = 0U;
static uint16_t unit_ticks = 100U;
static volatile uint32_t last_edge_tick = 0UL;
static volatile uint32_t pause_tick = 0UL;
static volatile uint8_t active_level = 0U;
/* Signed residual keeps the sum of encoded units phase-locked to Timer5. */
static int16_t quantization_residual_ticks = 0;

static inline uint32_t edge_timer5_now_from_isr(void)
{
    uint16_t high = timer5_high_word;
    uint16_t low = TCNT5;
    if (((TIFR5 & _BV(TOV5)) != 0U) && (low < 0x8000U))
    {
        high++;
    }
    return ((uint32_t)high << 16) | low;
}

static uint32_t edge_timer5_now(void)
{
    uint32_t result;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = edge_timer5_now_from_isr();
    }
    return result;
}

static inline bool edge_push_byte_from_isr(uint8_t value)
{
    uint16_t w = edge_write_sequence;
    if ((uint16_t)(w - edge_read_sequence) >= EDGE_FIFO_CAPACITY)
    {
        return false;
    }
    wav_sample_stream_isr_bytes[w & EDGE_FIFO_MASK] = value;
    asm volatile("" ::: "memory");
    edge_write_sequence = (uint16_t)(w + 1U);
    return true;
}

static inline bool edge_push_u32_interval_from_isr(uint8_t level, uint32_t units)
{
    uint16_t write_local = edge_write_sequence;
    uint16_t used = (uint16_t)(write_local - edge_read_sequence);

    if ((uint16_t)(EDGE_FIFO_CAPACITY - used) < EDGE_RECORD_EXTENDED_TOKEN_BYTES)
    {
        return false;
    }

    /* Publish all six bytes as one record so foreground never sees a split token. */
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = EDGE_RECORD_EXTENDED_TOKEN;
    write_local++;
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = level ? 1U : 0U;
    write_local++;
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = (uint8_t)(units & 0xFFUL);
    write_local++;
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = (uint8_t)((units >> 8) & 0xFFUL);
    write_local++;
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = (uint8_t)((units >> 16) & 0xFFUL);
    write_local++;
    wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = (uint8_t)((units >> 24) & 0xFFUL);
    write_local++;

    asm volatile("" ::: "memory");
    edge_write_sequence = write_local;
    return true;
}

/*
    Convert an exact Timer5 interval to an integer LEP/L16 duration while
    retaining the quantization remainder for the next edge.  Independent
    rounding causes a fixed frequency error for durations such as 500 us:
    500/16 = 31.25.  Error diffusion keeps the accumulated encoded time
    within half a unit of the captured time, so the long-term frequency is
    preserved.  The individual 16 us grid steps remain inherent to L16.
*/
static inline bool edge_emit_interval_from_isr(uint8_t level,
                                                uint32_t ticks)
{
    uint32_t corrected_ticks = ticks;
    uint32_t units;

    if (quantization_residual_ticks >= 0)
    {
        uint16_t residual = (uint16_t)quantization_residual_ticks;
        if (corrected_ticks <= (0xFFFFFFFFUL - residual))
        {
            corrected_ticks += residual;
        }
    }
    else
    {
        uint16_t residual = (uint16_t)(-quantization_residual_ticks);
        corrected_ticks = (corrected_ticks > residual) ?
            (corrected_ticks - residual) : 1UL;
    }

    units = (corrected_ticks + ((uint32_t)unit_ticks / 2UL)) /
            (uint32_t)unit_ticks;
    if (units == 0UL)
    {
        units = 1UL;
    }

    /* This difference is bounded to roughly +/- half a format unit. */
    {
        uint16_t remainder = (uint16_t)(corrected_ticks % (uint32_t)unit_ticks);
        if ((uint16_t)(remainder + (unit_ticks / 2U)) >= unit_ticks)
        {
            quantization_residual_ticks = (int16_t)remainder - (int16_t)unit_ticks;
        }
        else
        {
            quantization_residual_ticks = (int16_t)remainder;
        }
    }

    if (units <= 127UL)
    {
        int8_t encoded = level ? (int8_t)units : (int8_t)(-(int16_t)units);
        return edge_push_byte_from_isr((uint8_t)encoded);
    }

    return edge_push_u32_interval_from_isr(level, units);
}

static void edge_disable_pcint_from_isr(void)
{
    write_edge_monitor_stop();
}

static void edge_stop_timer_from_isr(void)
{
    edge_disable_pcint_from_isr();
    TIMSK5 &= (uint8_t)~_BV(TOIE5);
    TCCR5B = 0U;
    TCCR5A = 0U;
    TIFR5 = _BV(TOV5);
}

static void edge_enable_capture_from_isr(void)
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
        timer5_high_word = 0U;
        last_edge_tick = 0UL;
        pause_tick = 0UL;
        active_level = 0U;
        quantization_residual_ticks = 0;
        unit_ticks = 100U;
        edge_state = EDGE_RECORD_DRIVER_STOPPED;
    }
}

bool edge_record_driver_prepare(uint8_t unit_us)
{
    if ((unit_us != 16U) && (unit_us != 50U))
    {
        edge_state = EDGE_RECORD_DRIVER_BAD_ARGUMENT;
        return false;
    }

    edge_record_driver_init();

    /* Timer5 is F_CPU/8, so 16us=32 ticks and 50us=100 ticks. */
    unit_ticks = (uint16_t)((uint16_t)unit_us * 2U);
    edge_state = EDGE_RECORD_DRIVER_READY;
    return true;
}

bool edge_record_driver_start(void)
{
    uint32_t now;

    if (edge_state != EDGE_RECORD_DRIVER_READY)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        DDRJ &= (uint8_t)~_BV(PJ0);
        PORTJ &= (uint8_t)~_BV(PJ0);

        TCCR5A = 0U;
        TCCR5B = 0U;
        TCNT5 = 0U;
        TIFR5 = _BV(TOV5);
        timer5_high_word = 0U;
        TIMSK5 |= _BV(TOIE5);
        TCCR5B = _BV(CS51);

        now = edge_timer5_now_from_isr();
        active_level = mz_write_sample_from_isr();
        last_edge_tick = now;
        pause_tick = 0UL;
        edge_state = EDGE_RECORD_DRIVER_RUNNING;
        edge_enable_capture_from_isr();
    }

    return true;
}

bool edge_record_driver_pause(void)
{
    if (edge_state != EDGE_RECORD_DRIVER_RUNNING)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pause_tick = edge_timer5_now_from_isr();
        edge_disable_pcint_from_isr();
        edge_state = EDGE_RECORD_DRIVER_PAUSED;
    }
    return true;
}

bool edge_record_driver_resume(void)
{
    uint32_t now;
    uint8_t level;
    bool ok = true;

    if (edge_state != EDGE_RECORD_DRIVER_PAUSED)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        now = edge_timer5_now_from_isr();
        level = mz_write_sample_from_isr();

        if (level != active_level)
        {
            /* Simulate an edge exactly at the pause boundary. */
            if (!edge_emit_interval_from_isr(active_level,
                                             pause_tick - last_edge_tick))
            {
                edge_stop_timer_from_isr();
                edge_state = EDGE_RECORD_DRIVER_OVERRUN;
                ok = false;
            }
            else
            {
                active_level = level;
                last_edge_tick = now;
            }
        }
        else
        {
            /* Continue same pulse but remove paused time. */
            last_edge_tick += (now - pause_tick);
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
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint32_t stop_tick;
        bool valid = ((edge_state == EDGE_RECORD_DRIVER_RUNNING) ||
                      (edge_state == EDGE_RECORD_DRIVER_PAUSED));
        if (valid)
        {
            stop_tick = (edge_state == EDGE_RECORD_DRIVER_PAUSED) ?
                pause_tick : edge_timer5_now_from_isr();

            edge_disable_pcint_from_isr();
            if (!edge_emit_interval_from_isr(active_level,
                                             stop_tick - last_edge_tick))
            {
                edge_stop_timer_from_isr();
                edge_state = EDGE_RECORD_DRIVER_OVERRUN;
            }
            else
            {
                TIMSK5 &= (uint8_t)~_BV(TOIE5);
                TCCR5B = 0U;
                TCCR5A = 0U;
                TIFR5 = _BV(TOV5);
                edge_state = EDGE_RECORD_DRIVER_STOPPED;
            }
        }
    }
}

void edge_record_driver_abort(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        edge_stop_timer_from_isr();
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
    uint32_t now;
    uint32_t start;
    edge_record_driver_state_t state = edge_record_driver_get_state();

    if ((state != EDGE_RECORD_DRIVER_RUNNING) &&
        (state != EDGE_RECORD_DRIVER_PAUSED))
    {
        return 0UL;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        now = (state == EDGE_RECORD_DRIVER_PAUSED) ?
            pause_tick : edge_timer5_now_from_isr();
        start = last_edge_tick;
    }
    return now - start;
}

ISR(TIMER5_OVF_vect)
{
    timer5_high_word++;
}

void edge_record_driver_on_write_edge_from_isr(uint8_t new_level)
{
    uint32_t now;

    if (edge_state != EDGE_RECORD_DRIVER_RUNNING)
    {
        return;
    }

    if (new_level == active_level)
    {
        return;
    }

    now = edge_timer5_now_from_isr();
    if (!edge_emit_interval_from_isr(active_level, now - last_edge_tick))
    {
        edge_stop_timer_from_isr();
        edge_state = EDGE_RECORD_DRIVER_OVERRUN;
        return;
    }

    active_level = new_level;
    last_edge_tick = now;
}
