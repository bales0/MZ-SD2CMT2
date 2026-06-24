#include "wav_playback_driver.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "mzio.h"
#include "../streams/wav_sample_stream.h"

#if !defined(TIMER1_COMPA_vect)
#error "WAV playback driver requires Timer1 compare A on Arduino Mega 2560"
#endif

/*
    Fixed sample-by-sample Timer1 output.

    The ring is accessed only when one packed byte has been exhausted. The
    current byte and its remaining valid-bit count stay in local driver state,
    so the 44.1/48 kHz ISR avoids a volatile 16-bit FIFO operation on seven of
    every eight sample ticks.
*/
static volatile uint8_t driver_state = WAV_PLAYBACK_DRIVER_STOPPED;

static uint16_t timer_sample_rate = 0;
static uint16_t timer_base_ticks = 0;
static uint16_t timer_remainder_ticks = 0;
static uint16_t timer_phase_ticks = 0;

static uint8_t cached_output_byte = 0;
static uint8_t cached_output_bits = 0;

static volatile uint16_t emitted_sequence = 0;

static inline __attribute__((always_inline))
void wav_playback_schedule_period_from_isr(void)
{
    uint16_t ticks = timer_base_ticks;
    uint16_t phase = timer_phase_ticks;
    uint16_t threshold = (uint16_t)(
        timer_sample_rate - timer_remainder_ticks
    );

    /* phase = (phase + remainder) mod sample_rate. */
    if (phase >= threshold)
    {
        phase = (uint16_t)(phase - threshold);
        ticks++;
    }
    else
    {
        phase = (uint16_t)(phase + timer_remainder_ticks);
    }

    timer_phase_ticks = phase;
    OCR1A = (uint16_t)(ticks - 1U);
}

static inline __attribute__((always_inline))
void wav_playback_disable_timer_from_isr(void)
{
    TCCR1B = _BV(WGM12);
    TIMSK1 &= (uint8_t)~_BV(OCIE1A);
    TIFR1 = _BV(OCF1A);

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
    TCCR1A = 0;
    PORTB &= (uint8_t)~_BV(PB5);
#endif
}

static void wav_playback_disable_timer_from_foreground(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCCR1B = _BV(WGM12);
        TIMSK1 &= (uint8_t)~_BV(OCIE1A);
        TIFR1 = _BV(OCF1A);

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
        TCCR1A = 0;
        PORTB &= (uint8_t)~_BV(PB5);
#else
        TCCR1A = 0;
#endif
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

static void wav_playback_enable_timer_from_foreground(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCNT1 = 0;
        TIFR1 = _BV(OCF1A);

#if WAV_PLAYBACK_TIMING_MARKER_ENABLE
        TCCR1A = _BV(COM1A0);
#else
        TCCR1A = 0;
#endif

        driver_state = WAV_PLAYBACK_DRIVER_RUNNING;
        TIMSK1 |= _BV(OCIE1A);
        TCCR1B = _BV(WGM12) | _BV(CS10);
    }
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
bool wav_playback_emit_one_sample_from_isr(void)
{
    uint8_t level;

    if (cached_output_bits == 0U)
    {
        if (!wav_playback_load_cached_byte_from_isr())
        {
            return false;
        }
    }

    level = (uint8_t)(cached_output_byte & 1U);
    cached_output_byte >>= 1U;
    cached_output_bits--;

    mz_read_set_fast_from_isr(level);
    emitted_sequence = (uint16_t)(emitted_sequence + 1U);
    return true;
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
        TCCR1A = 0;
        TCCR1B = _BV(WGM12);
        TCNT1 = 0;
        OCR1A = 0xFFFFU;
        TIMSK1 &= (uint8_t)~_BV(OCIE1A);
        TIFR1 = _BV(OCF1A);
    }

    timer_sample_rate = 0;
    timer_base_ticks = 0;
    timer_remainder_ticks = 0;
    timer_phase_ticks = 0;

    cached_output_byte = 0;
    cached_output_bits = 0;
    emitted_sequence = 0;
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

    if ((base_ticks == 0UL) || (base_ticks > 65535UL))
    {
        driver_state = WAV_PLAYBACK_DRIVER_BAD_RATE;
        return false;
    }

    wav_playback_disable_timer_from_foreground();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCNT1 = 0;
        timer_sample_rate = (uint16_t)sample_rate;
        timer_base_ticks = (uint16_t)base_ticks;
        timer_remainder_ticks = (uint16_t)remainder_ticks;
        timer_phase_ticks = 0;
        cached_output_byte = 0;
        cached_output_bits = 0;
        emitted_sequence = 0;
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

    if (!wav_playback_emit_one_sample_from_isr())
    {
        driver_state = wav_sample_stream_finished_fast_from_isr() ?
            WAV_PLAYBACK_DRIVER_FINISHED : WAV_PLAYBACK_DRIVER_UNDERRUN;
        return false;
    }

    wav_playback_schedule_period_from_isr();
    wav_playback_enable_timer_from_foreground();
    return true;
}

bool wav_playback_driver_pause(void)
{
    if (wav_playback_state_snapshot() != WAV_PLAYBACK_DRIVER_RUNNING)
    {
        return false;
    }

    wav_playback_disable_timer_from_foreground();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        driver_state = WAV_PLAYBACK_DRIVER_PAUSED;
    }

    return true;
}

bool wav_playback_driver_resume(void)
{
    if (wav_playback_state_snapshot() != WAV_PLAYBACK_DRIVER_PAUSED)
    {
        return false;
    }

    wav_playback_schedule_period_from_isr();
    wav_playback_enable_timer_from_foreground();
    return true;
}

void wav_playback_driver_stop(void)
{
    wav_playback_disable_timer_from_foreground();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        driver_state = WAV_PLAYBACK_DRIVER_STOPPED;
        cached_output_byte = 0;
        cached_output_bits = 0;
        emitted_sequence = 0;
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

ISR(TIMER1_COMPA_vect)
{
    if (driver_state != WAV_PLAYBACK_DRIVER_RUNNING)
    {
        wav_playback_disable_timer_from_isr();
        driver_state = WAV_PLAYBACK_DRIVER_BAD_ARGUMENT;
        return;
    }

    if (!wav_playback_emit_one_sample_from_isr())
    {
        wav_playback_disable_timer_from_isr();
        mz_read_set_fast_from_isr(0U);

        driver_state = wav_sample_stream_finished_fast_from_isr() ?
            WAV_PLAYBACK_DRIVER_FINISHED : WAV_PLAYBACK_DRIVER_UNDERRUN;
        return;
    }

    wav_playback_schedule_period_from_isr();
}
