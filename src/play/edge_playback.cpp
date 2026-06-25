#include "edge_playback.h"
#include "mzf_playback.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

#include "../drivers/mzio.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"

#if !defined(TIMER3_COMPB_vect)
#error "EDGE playback requires Timer3 compare-B on ATmega2560"
#endif

#define EDGE_FIFO_BYTES WAV_SAMPLE_STREAM_BUFFER_BYTES
#define EDGE_FIFO_CAPACITY (EDGE_FIFO_BYTES - 1U)
#define EDGE_FIFO_MASK (EDGE_FIFO_BYTES - 1U)
#define EDGE_WORK_BYTES WAV_SAMPLE_STREAM_REFILL_BLOCK

static volatile uint16_t edge_read_sequence = 0U;
static volatile uint16_t edge_write_sequence = 0U;
static volatile uint8_t edge_source_finished = 1U;
static volatile uint8_t edge_state = EDGE_PLAYBACK_STOPPED;

static uint8_t edge_unit_us = 50U;
static uint16_t edge_unit_ticks = 800U;
static bool edge_invert = false;
static uint32_t edge_paused_remaining_ticks = 0UL;
/* Timer3 compare is 16-bit: a source slot may be split into safe chunks. */
static uint32_t edge_slot_remaining_ticks = 0UL;
#define EDGE_TIMER_MAX_CHUNK_TICKS 60000UL
static uint32_t edge_total_bytes = 0UL;
static volatile uint32_t edge_consumed_bytes = 0UL;
static char edge_error_text[17];

static void edge_set_error(const char *text, edge_playback_state_t state)
{
    if (text == NULL)
    {
        text = "EDGE ERROR";
    }
    strncpy(edge_error_text, text, sizeof(edge_error_text) - 1U);
    edge_error_text[sizeof(edge_error_text) - 1U] = '\0';
    edge_state = (uint8_t)state;
}

static uint16_t edge_used_snapshot(void)
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

static bool edge_pop_from_isr(int8_t *value)
{
    uint16_t r = edge_read_sequence;
    if ((value == NULL) || (r == edge_write_sequence))
    {
        return false;
    }
    *value = (int8_t)wav_sample_stream_isr_bytes[r & EDGE_FIFO_MASK];
    edge_read_sequence = (uint16_t)(r + 1U);
    edge_consumed_bytes++;
    return true;
}

static void edge_stop_timer_from_isr(void)
{
    TIMSK3 &= (uint8_t)~_BV(OCIE3B);
    TCCR3A = 0U;
    TCCR3B = 0U;
    TIFR3 = _BV(OCF3B);
}

static void edge_stop_timer_from_foreground(bool force_low)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TIMSK3 &= (uint8_t)~_BV(OCIE3B);
        TCCR3A = 0U;
        TCCR3B = 0U;
        TIFR3 = _BV(OCF3B);
    }
    if (force_low)
    {
        mz_read_set(false);
    }
}

static uint16_t edge_chunk_ticks(uint32_t ticks)
{
    if (ticks == 0UL)
    {
        return 1U;
    }
    return (ticks > EDGE_TIMER_MAX_CHUNK_TICKS) ?
        (uint16_t)EDGE_TIMER_MAX_CHUNK_TICKS : (uint16_t)ticks;
}

/* First slot after start/resume is anchored to the current timer position. */
static void edge_schedule_first_chunk_from_isr(uint32_t ticks)
{
    uint16_t chunk = edge_chunk_ticks(ticks);
    edge_slot_remaining_ticks = ticks - (uint32_t)chunk;
    OCR3B = (uint16_t)(TCNT3 + chunk);
}

/* Subsequent chunks remain anchored to the preceding compare instant. */
static void edge_schedule_next_chunk_from_isr(void)
{
    uint16_t chunk = edge_chunk_ticks(edge_slot_remaining_ticks);
    edge_slot_remaining_ticks -= (uint32_t)chunk;
    OCR3B = (uint16_t)(OCR3B + chunk);
}

static uint32_t edge_set_slot_level_from_isr(int8_t slot)
{
    uint8_t magnitude = (slot < 0) ? (uint8_t)(-slot) : (uint8_t)slot;
    uint8_t level = (slot > 0) ? 1U : 0U;

    if (edge_invert)
    {
        level ^= 1U;
    }

    mz_read_set_fast_from_isr(level);
    return (uint32_t)magnitude * (uint32_t)edge_unit_ticks;
}

static bool edge_refill_once(void)
{
    uint16_t used;
    uint16_t free_bytes;
    uint16_t request;
    int16_t received;
    uint16_t write_local;
    uint8_t *work;

    if (edge_source_finished != 0U)
    {
        return true;
    }

    used = edge_used_snapshot();
    if (used > EDGE_FIFO_CAPACITY)
    {
        edge_set_error("EDGE FIFO", EDGE_PLAYBACK_IO_ERROR);
        return false;
    }

    free_bytes = (uint16_t)(EDGE_FIFO_CAPACITY - used);
    if (free_bytes == 0U)
    {
        return true;
    }

    request = free_bytes;
    if (request > EDGE_WORK_BYTES)
    {
        request = EDGE_WORK_BYTES;
    }

    work = wav_sample_stream_get_shared_work_buffer();
    received = sdcard_file_read(work, request);
    if (received < 0)
    {
        edge_set_error("EDGE READ", EDGE_PLAYBACK_IO_ERROR);
        return false;
    }

    if (received == 0)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            edge_source_finished = 1U;
        }
        return true;
    }

    write_local = edge_write_sequence;
    for (int16_t i = 0; i < received; ++i)
    {
        wav_sample_stream_isr_bytes[write_local & EDGE_FIFO_MASK] = work[i];
        write_local = (uint16_t)(write_local + 1U);
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        asm volatile("" ::: "memory");
        edge_write_sequence = write_local;
        if (sdcard_file_position() >= edge_total_bytes)
        {
            edge_source_finished = 1U;
        }
    }

    return true;
}

static bool edge_prefill(void)
{
    while (edge_source_finished == 0U)
    {
        uint16_t before = edge_used_snapshot();
        if (!edge_refill_once())
        {
            return false;
        }
        if (edge_used_snapshot() == before)
        {
            break;
        }
    }
    return edge_used_snapshot() != 0U;
}

void edge_playback_init(void)
{
    edge_stop_timer_from_foreground(true);
    edge_read_sequence = 0U;
    edge_write_sequence = 0U;
    edge_source_finished = 1U;
    edge_state = EDGE_PLAYBACK_STOPPED;
    edge_unit_us = 50U;
    edge_unit_ticks = 800U;
    edge_invert = false;
    edge_paused_remaining_ticks = 0UL;
    edge_slot_remaining_ticks = 0UL;
    edge_total_bytes = 0UL;
    edge_consumed_bytes = 0UL;
    edge_error_text[0] = '\0';
}

bool edge_playback_prepare(const char *path, uint8_t unit_us, bool invert)
{
    edge_playback_stop();

    if ((path == NULL) || ((unit_us != 16U) && (unit_us != 50U)))
    {
        edge_set_error("EDGE ARG", EDGE_PLAYBACK_BAD_FILE);
        return false;
    }

    /* Timer3 runs at 16 MHz, exact for both 16us and 50us units. */
    edge_unit_us = unit_us;
    edge_unit_ticks = (uint16_t)((uint16_t)unit_us * 16U);
    edge_invert = invert;
    edge_error_text[0] = '\0';

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        edge_read_sequence = 0U;
        edge_write_sequence = 0U;
        edge_source_finished = 0U;
        edge_consumed_bytes = 0UL;
    }

    if (!sdcard_file_open_read(path))
    {
        edge_set_error("EDGE OPEN", EDGE_PLAYBACK_IO_ERROR);
        return false;
    }

    edge_total_bytes = sdcard_file_size();
    if ((edge_total_bytes == 0UL) || !edge_prefill())
    {
        sdcard_file_close();
        edge_set_error("EMPTY EDGE", EDGE_PLAYBACK_BAD_FILE);
        return false;
    }

    edge_state = EDGE_PLAYBACK_READY;
    return true;
}

bool edge_playback_start(void)
{
    int8_t first;

    if (edge_state != EDGE_PLAYBACK_READY)
    {
        return false;
    }

    if (!edge_pop_from_isr(&first) || (first == 0))
    {
        edge_set_error("EDGE FIRST", EDGE_PLAYBACK_BAD_FILE);
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCNT3 = 0U;
        TCCR3A = 0U;
        TCCR3B = _BV(CS30);
        TIFR3 = _BV(OCF3B);
        edge_state = EDGE_PLAYBACK_RUNNING;
        edge_schedule_first_chunk_from_isr(edge_set_slot_level_from_isr(first));
        TIMSK3 |= _BV(OCIE3B);
    }

    mz_sense_set(false);
    return true;
}

bool edge_playback_pause(void)
{
    if (edge_state != EDGE_PLAYBACK_RUNNING)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint16_t current_chunk_remaining = (uint16_t)(OCR3B - TCNT3);
        if ((TIFR3 & _BV(OCF3B)) != 0U || current_chunk_remaining == 0U)
        {
            current_chunk_remaining = 1U;
        }
        edge_paused_remaining_ticks = (uint32_t)current_chunk_remaining +
                                      edge_slot_remaining_ticks;
        edge_slot_remaining_ticks = 0UL;
        edge_stop_timer_from_isr();
        edge_state = EDGE_PLAYBACK_PAUSED;
    }
    return true;
}

bool edge_playback_resume(void)
{
    if (edge_state != EDGE_PLAYBACK_PAUSED)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCNT3 = 0U;
        TCCR3A = 0U;
        TCCR3B = _BV(CS30);
        TIFR3 = _BV(OCF3B);
        edge_state = EDGE_PLAYBACK_RUNNING;
        edge_schedule_first_chunk_from_isr(edge_paused_remaining_ticks);
        TIMSK3 |= _BV(OCIE3B);
    }
    return true;
}

void edge_playback_stop(void)
{
    edge_stop_timer_from_foreground(true);
    sdcard_file_close();
    edge_source_finished = 1U;
    edge_state = EDGE_PLAYBACK_STOPPED;
    edge_read_sequence = 0U;
    edge_write_sequence = 0U;
    edge_paused_remaining_ticks = 0UL;
    edge_slot_remaining_ticks = 0UL;
    mz_sense_set(true);
}

void edge_playback_service(void)
{
    if (edge_state == EDGE_PLAYBACK_RUNNING)
    {
        if ((edge_source_finished == 0U) &&
            (edge_used_snapshot() <= EDGE_WORK_BYTES))
        {
            (void)edge_refill_once();
        }
    }

    if ((edge_state == EDGE_PLAYBACK_FINISHED) ||
        (edge_state == EDGE_PLAYBACK_UNDERRUN) ||
        (edge_state == EDGE_PLAYBACK_IO_ERROR) ||
        (edge_state == EDGE_PLAYBACK_BAD_FILE))
    {
        sdcard_file_close();
        mz_sense_set(true);
    }
}

edge_playback_state_t edge_playback_get_state(void)
{
    uint8_t state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        state = edge_state;
    }
    return (edge_playback_state_t)state;
}

const char *edge_playback_get_error_text(void) { return edge_error_text; }

uint8_t edge_playback_get_buffer_fill_percent(void)
{
    uint32_t percent = ((uint32_t)edge_used_snapshot() * 100UL) / EDGE_FIFO_CAPACITY;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

uint32_t edge_playback_get_consumed_bytes(void)
{
    uint32_t result;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = edge_consumed_bytes;
    }
    return result;
}

uint32_t edge_playback_get_total_bytes(void) { return edge_total_bytes; }

/*
    Timer3 compare-B has exactly one hardware vector in the complete firmware.
    LEP/L16 and MZF/MZT both use it, but play_engine guarantees that only one
    source is running at a time.  Keep the vector here and let the MZF module
    explicitly claim a compare event only while its NORMAL PWM transport owns
    Timer3.  ULTRA FAST itself is foreground handshake code and never claims
    Timer3 compare-B.
*/
static void edge_playback_timer3_compb_from_isr(void)
{
    int8_t next;

    if (edge_state != EDGE_PLAYBACK_RUNNING)
    {
        edge_stop_timer_from_isr();
        return;
    }

    /* A LEP zero can mean 101600 Timer3 ticks.  Complete its chunks first. */
    if (edge_slot_remaining_ticks != 0UL)
    {
        edge_schedule_next_chunk_from_isr();
        return;
    }

    if (!edge_pop_from_isr(&next))
    {
        edge_stop_timer_from_isr();
        mz_read_set_fast_from_isr(0U);
        edge_state = (edge_source_finished != 0U) ?
            EDGE_PLAYBACK_FINISHED : EDGE_PLAYBACK_UNDERRUN;
        return;
    }

    if (next == 0)
    {
        /* 0 extends the current polarity by exactly 127 format units. */
        edge_slot_remaining_ticks = (uint32_t)127U * (uint32_t)edge_unit_ticks;
        edge_schedule_next_chunk_from_isr();
        return;
    }

    edge_slot_remaining_ticks = edge_set_slot_level_from_isr(next);
    edge_schedule_next_chunk_from_isr();
}

ISR(TIMER3_COMPB_vect)
{
    if (mzf_playback_timer3_compb_from_isr())
    {
        return;
    }

    edge_playback_timer3_compb_from_isr();
}
