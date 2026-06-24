#include "wav_playback_driver.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "mzio.h"
#include "../streams/wav_sample_stream.h"

#if !defined(TIMER3_COMPA_vect)
#error "WAV playback driver requires Timer3 compare A on Arduino Mega 2560"
#endif

#if (WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS < 64U)
#error "Timer3 prepare lead must be at least 64 CPU ticks"
#endif

/*
    Timer3 runs in normal mode, no prescaler:

      OCR3A = foreground preparation interrupt, 192 clocks before an edge.
      OCR3B = D2/OC3B hardware set or clear at the sample boundary.
      OCR3C = optional D3/OC3C hardware marker at the same boundary.

    This separates the non-deterministic ISR entry latency from the physical
    READ transition. The ISR has 12 us of lead time; the D2 change itself is
    performed by Timer3 hardware, not by a CPU instruction in the ISR.
*/
static volatile uint8_t driver_state = WAV_PLAYBACK_DRIVER_STOPPED;

static uint16_t timer_sample_rate = 0;
static uint16_t timer_base_ticks = 0;
static uint16_t timer_remainder_ticks = 0;
static uint16_t timer_phase_ticks = 0;

static uint16_t next_edge_tick = 0;
static uint16_t armed_edge_tick = 0;
static uint8_t armed_level = 0;
static uint8_t armed_output_pending = 0;

/* A sample scheduled before PAUSE but not yet output must be replayed. */
static uint8_t resume_forced_level = 0;
static uint8_t resume_forced_valid = 0;
static uint8_t resume_forced_counted = 0;

/* Non-zero means the next Compare-A event stops after hardware final clear. */
static uint8_t terminal_state_after_clear = WAV_PLAYBACK_DRIVER_STOPPED;

static uint8_t cached_output_byte = 0;
static uint8_t cached_output_bits = 0;

static volatile uint16_t emitted_sequence = 0;

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
#define TIMER3_TCCRA_MARKER_BITS _BV(COM3C0)
#else
#define TIMER3_TCCRA_MARKER_BITS 0U
#endif

static inline __attribute__((always_inline))
bool wav_playback_time_before(uint16_t left, uint16_t right)
{
    return ((int16_t)(left - right) < 0);
}

static inline __attribute__((always_inline))
uint16_t wav_playback_next_period_from_isr(void)
{
    uint16_t ticks = timer_base_ticks;
    uint16_t phase = timer_phase_ticks;

    phase = (uint16_t)(phase + timer_remainder_ticks);

    if (phase >= timer_sample_rate)
    {
        phase = (uint16_t)(phase - timer_sample_rate);
        ticks++;
    }

    timer_phase_ticks = phase;
    return ticks;
}

static inline __attribute__((always_inline))
void wav_playback_set_hardware_level_from_isr(uint8_t level,
                                              uint16_t edge_tick)
{
    /*
        Non-PWM Timer3 OC3B mode:
          COM3B=10 -> clear D2 on Compare-B
          COM3B=11 -> set   D2 on Compare-B
    */
    uint8_t control = (uint8_t)(TIMER3_TCCRA_MARKER_BITS | _BV(COM3B1));

    if (level & 1U)
    {
        control |= _BV(COM3B0);
    }

    TCCR3A = control;
    OCR3B = edge_tick;

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
    OCR3C = edge_tick;
#endif
}

static inline __attribute__((always_inline))
void wav_playback_schedule_prepare_from_isr(uint16_t edge_tick)
{
    OCR3A = (uint16_t)(edge_tick - WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS);
}

static inline __attribute__((always_inline))
void wav_playback_disable_timer_from_isr(void)
{
    TIMSK3 &= (uint8_t)~_BV(OCIE3A);
    TCCR3B = 0U;
    TCCR3A = 0U;
    TIFR3 = (uint8_t)(_BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C));

    /* D2 and optional D3 become normal GPIO outputs and idle LOW. */
    PORTE &= (uint8_t)~(_BV(PE4) | _BV(PE5));
}

static void wav_playback_disable_timer_from_foreground(bool hold_read_level)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TIMSK3 &= (uint8_t)~_BV(OCIE3A);
        TCCR3B = 0U;

        if (!hold_read_level)
        {
            TCCR3A = 0U;
            PORTE &= (uint8_t)~(_BV(PE4) | _BV(PE5));
        }

        TIFR3 = (uint8_t)(_BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C));
    }
}

static wav_playback_driver_state_t wav_playback_state_snapshot(void)
{
    uint8_t state;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        state = driver_state;
    }

    return (wav_playback_driver_state_t)state;
}

static inline __attribute__((always_inline))
bool wav_playback_load_cached_byte_from_isr(void)
{
    uint8_t valid_bits;

    if (!wav_sample_stream_pop_byte_fast_from_isr(
            &cached_output_byte, &valid_bits))
    {
        return false;
    }

    cached_output_bits = valid_bits;
    return true;
}

static inline __attribute__((always_inline))
bool wav_playback_take_next_level_from_isr(uint8_t *level,
                                           bool *already_counted)
{
    if ((level == NULL) || (already_counted == NULL))
    {
        return false;
    }

    if (resume_forced_valid != 0U)
    {
        *level = resume_forced_level;
        *already_counted = (resume_forced_counted != 0U);
        resume_forced_valid = 0U;
        resume_forced_counted = 0U;
        return true;
    }

    if (cached_output_bits == 0U)
    {
        if (!wav_playback_load_cached_byte_from_isr())
        {
            return false;
        }
    }

    *level = (uint8_t)(cached_output_byte & 1U);
    cached_output_byte >>= 1U;
    cached_output_bits--;
    *already_counted = false;
    return true;
}

static inline __attribute__((always_inline))
void wav_playback_finish_after_current_sample_from_isr(uint8_t final_state)
{
    /*
        We are at E(n)-lead and have no sample n. The previous sample has
        already been scheduled for the interval ending at E(n). Program a
        hardware LOW exactly at E(n), then stop one CPU tick later.
    */
    wav_playback_set_hardware_level_from_isr(0U, next_edge_tick);
    terminal_state_after_clear = final_state;
    OCR3A = (uint16_t)(next_edge_tick + 1U);
}

static void wav_playback_start_timer_from_foreground(void)
{
    uint16_t first_edge;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        /*
            384 ticks from a stopped counter gives the first Compare-A event
            192 ticks before the first physical D2 sample boundary.
        */
        TCNT3 = 0U;
        first_edge = (uint16_t)(
            WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS * 2U
        );

        next_edge_tick = first_edge;
        armed_edge_tick = 0U;
        armed_level = 0U;
        armed_output_pending = 0U;
        terminal_state_after_clear = WAV_PLAYBACK_DRIVER_STOPPED;

        TCCR3A = TIMER3_TCCRA_MARKER_BITS;
        OCR3A = (uint16_t)(first_edge - WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS);
        OCR3B = 0xFFFFU;

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
        OCR3C = 0xFFFFU;
#endif

        TIFR3 = (uint8_t)(_BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C));
        driver_state = WAV_PLAYBACK_DRIVER_RUNNING;
        TIMSK3 |= _BV(OCIE3A);
        TCCR3B = _BV(CS30);
    }
}

void wav_playback_driver_init(void)
{
    mzio_init();

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
    pinMode(WAV_PLAYBACK_TIMING_MARKER_PIN, OUTPUT);
    digitalWrite(WAV_PLAYBACK_TIMING_MARKER_PIN, LOW);
#endif

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCCR3A = 0U;
        TCCR3B = 0U;
        TCNT3 = 0U;
        OCR3A = 0xFFFFU;
        OCR3B = 0xFFFFU;

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
        OCR3C = 0xFFFFU;
#endif

        TIMSK3 &= (uint8_t)~_BV(OCIE3A);
        TIFR3 = (uint8_t)(_BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C));
    }

    timer_sample_rate = 0U;
    timer_base_ticks = 0U;
    timer_remainder_ticks = 0U;
    timer_phase_ticks = 0U;
    next_edge_tick = 0U;
    armed_edge_tick = 0U;
    armed_level = 0U;
    armed_output_pending = 0U;
    resume_forced_level = 0U;
    resume_forced_valid = 0U;
    resume_forced_counted = 0U;
    terminal_state_after_clear = WAV_PLAYBACK_DRIVER_STOPPED;
    cached_output_byte = 0U;
    cached_output_bits = 0U;
    emitted_sequence = 0U;
    driver_state = WAV_PLAYBACK_DRIVER_STOPPED;
}

bool wav_playback_driver_prepare(uint32_t sample_rate)
{
    uint32_t base_ticks;
    uint32_t remainder_ticks;

    if ((sample_rate == 0UL) || (sample_rate > 65535UL))
    {
        driver_state = WAV_PLAYBACK_DRIVER_BAD_ARGUMENT;
        return false;
    }

    base_ticks = (uint32_t)F_CPU / sample_rate;
    remainder_ticks = (uint32_t)F_CPU % sample_rate;

    if ((base_ticks <= WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS) ||
        (base_ticks > 65535UL))
    {
        driver_state = WAV_PLAYBACK_DRIVER_BAD_RATE;
        return false;
    }

    wav_playback_disable_timer_from_foreground(false);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCNT3 = 0U;
        timer_sample_rate = (uint16_t)sample_rate;
        timer_base_ticks = (uint16_t)base_ticks;
        timer_remainder_ticks = (uint16_t)remainder_ticks;
        timer_phase_ticks = 0U;
        next_edge_tick = 0U;
        armed_edge_tick = 0U;
        armed_level = 0U;
        armed_output_pending = 0U;
        resume_forced_level = 0U;
        resume_forced_valid = 0U;
        resume_forced_counted = 0U;
        terminal_state_after_clear = WAV_PLAYBACK_DRIVER_STOPPED;
        cached_output_byte = 0U;
        cached_output_bits = 0U;
        emitted_sequence = 0U;
        driver_state = WAV_PLAYBACK_DRIVER_READY;
    }

    mz_read_set(false);
    return true;
}

bool wav_playback_driver_start(void)
{
    if (wav_playback_state_snapshot() != WAV_PLAYBACK_DRIVER_READY)
    {
        return false;
    }

    if (wav_sample_stream_available() == 0U)
    {
        driver_state = wav_sample_stream_finished_fast_from_isr() ?
            WAV_PLAYBACK_DRIVER_FINISHED : WAV_PLAYBACK_DRIVER_UNDERRUN;
        return false;
    }

    wav_playback_start_timer_from_foreground();
    return true;
}

bool wav_playback_driver_pause(void)
{
    uint16_t now;

    if (wav_playback_state_snapshot() != WAV_PLAYBACK_DRIVER_RUNNING)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        now = TCNT3;

        /*
            If Compare-A already consumed a sample but Compare-B has not
            physically applied it yet, preserve that exact sample for resume.
        */
        if ((armed_output_pending != 0U) &&
            wav_playback_time_before(now, armed_edge_tick))
        {
            resume_forced_level = armed_level;
            resume_forced_valid = 1U;
            resume_forced_counted = 1U;
        }

        armed_output_pending = 0U;
        TIMSK3 &= (uint8_t)~_BV(OCIE3A);
        TCCR3B = 0U;
        TIFR3 = (uint8_t)(_BV(OCF3A) | _BV(OCF3B) | _BV(OCF3C));
        driver_state = WAV_PLAYBACK_DRIVER_PAUSED;
    }

    /* Keep COM3B connected so D2 holds its last physical level. */
    return true;
}

bool wav_playback_driver_resume(void)
{
    if (wav_playback_state_snapshot() != WAV_PLAYBACK_DRIVER_PAUSED)
    {
        return false;
    }

    wav_playback_start_timer_from_foreground();
    return true;
}

void wav_playback_driver_stop(void)
{
    wav_playback_disable_timer_from_foreground(false);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        driver_state = WAV_PLAYBACK_DRIVER_STOPPED;
        next_edge_tick = 0U;
        armed_edge_tick = 0U;
        armed_level = 0U;
        armed_output_pending = 0U;
        resume_forced_level = 0U;
        resume_forced_valid = 0U;
        resume_forced_counted = 0U;
        terminal_state_after_clear = WAV_PLAYBACK_DRIVER_STOPPED;
        cached_output_byte = 0U;
        cached_output_bits = 0U;
        emitted_sequence = 0U;
    }

    mz_read_set(false);
}

wav_playback_driver_state_t wav_playback_driver_get_state(void)
{
    return wav_playback_state_snapshot();
}

uint32_t wav_playback_driver_get_sample_rate(void)
{
    return (uint32_t)timer_sample_rate;
}

uint8_t wav_playback_driver_get_read_pin(void)
{
    return mzio_read_pin();
}

uint16_t wav_playback_driver_get_emitted_sequence(void)
{
    uint16_t sequence;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        sequence = emitted_sequence;
    }

    return sequence;
}

uint16_t wav_playback_driver_get_jitter_ticks(void)
{
    return 0U;
}

ISR(TIMER3_COMPA_vect)
{
    uint8_t level;
    bool already_counted;
    uint16_t period;

    if (driver_state != WAV_PLAYBACK_DRIVER_RUNNING)
    {
        wav_playback_disable_timer_from_isr();
        driver_state = WAV_PLAYBACK_DRIVER_BAD_ARGUMENT;
        return;
    }

    if (terminal_state_after_clear != WAV_PLAYBACK_DRIVER_STOPPED)
    {
        uint8_t final_state = terminal_state_after_clear;

        wav_playback_disable_timer_from_isr();
        driver_state = final_state;
        return;
    }

    if (!wav_playback_take_next_level_from_isr(&level, &already_counted))
    {
        wav_playback_finish_after_current_sample_from_isr(
            wav_sample_stream_finished_fast_from_isr() ?
                WAV_PLAYBACK_DRIVER_FINISHED : WAV_PLAYBACK_DRIVER_UNDERRUN
        );
        return;
    }

    armed_level = level;
    armed_edge_tick = next_edge_tick;
    armed_output_pending = 1U;

    wav_playback_set_hardware_level_from_isr(level, next_edge_tick);

    if (!already_counted)
    {
        emitted_sequence = (uint16_t)(emitted_sequence + 1U);
    }

    period = wav_playback_next_period_from_isr();
    next_edge_tick = (uint16_t)(next_edge_tick + period);
    wav_playback_schedule_prepare_from_isr(next_edge_tick);
}
