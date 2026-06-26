#include "mzf_playback.h"
#include "timer3b_owner.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

#include "../drivers/mzio.h"
#include "../drivers/flash_text.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"

#if !defined(TIMER3_COMPB_vect)
#error "MZF playback requires Timer3 compare-B on ATmega2560"
#endif

/*
    The binary MZF/M12 header is 128 bytes. MZT is handled as a sequence of
    header + declared data-block records, each separated by the MZ MOTOR
    pause/restart cycle expected by the original monitor routine.
*/
#define MZF_HEADER_BYTES 128U
#define MZF_HEADER_DATA_LENGTH_OFFSET 0x12U

/*
    MZ-800 monitor waveform, matched to the validated reference WAV:

      short / logical 0: HIGH 250 us, LOW 250 us  (500 us total)
      long  / logical 1: HIGH 500 us, LOW 500 us (1000 us total)

    The MZ-800 ``sane'' tape framing sends a 6,400-short-pulse leader before
    both the header and the data section.  The old
    MZ-700-style 480/680 us profile produced an output stream about 26 % too
    short for the MZ-800 reference format.
*/
#define MZF_TICKS_PER_US ((uint16_t)(F_CPU / 1000000UL))
#define MZF_US_TO_TICKS(us) ((uint16_t)((uint32_t)(us) * MZF_TICKS_PER_US))
#define MZF_SHORT_HIGH_TICKS MZF_US_TO_TICKS(250U)
#define MZF_SHORT_LOW_TICKS  MZF_US_TO_TICKS(250U)
#define MZF_LONG_HIGH_TICKS  MZF_US_TO_TICKS(500U)
#define MZF_LONG_LOW_TICKS   MZF_US_TO_TICKS(500U)

#define MZF_MZ800_LONG_GAP_SHORT_PULSES 6400UL
#define MZF_MZ800_SHORT_GAP_SHORT_PULSES 6400UL
#define MZF_MZ800_LONG_MARK_LONG_PULSES 40UL
#define MZF_MZ800_LONG_MARK_SHORT_PULSES 40UL
#define MZF_MZ800_SHORT_MARK_LONG_PULSES 20UL
#define MZF_MZ800_SHORT_MARK_SHORT_PULSES 20UL
#define MZF_MZ800_TAPE_MARK_FINAL_LONG_PULSES 2UL
#define MZF_MZ800_TRAILING_LONG_PULSES 2UL

#define MZF_FIFO_BYTES WAV_SAMPLE_STREAM_BUFFER_BYTES
#define MZF_FIFO_CAPACITY (MZF_FIFO_BYTES - 1U)
#define MZF_FIFO_MASK (MZF_FIFO_BYTES - 1U)
#define MZF_REFILL_BLOCK WAV_SAMPLE_STREAM_REFILL_BLOCK
/*
    Header/data boundaries normally use an MZ MOTOR off/on cycle.  Some
    transports, and PLAY CTRL / MANUAL, leave MOTOR high; do not strand a
    binary image after its 128-byte header in that case.
*/
#define MZF_BOUNDARY_AUTO_CONTINUE_MS 120UL

#if ((MZF_FIFO_BYTES & (MZF_FIFO_BYTES - 1U)) != 0U)
#error "MZF FIFO needs a power-of-two size"
#endif

typedef enum
{
    MZF_STAGE_NONE = 0,
    MZF_STAGE_HEADER,
    MZF_STAGE_DATA
} mzf_stage_t;

typedef enum
{
    MZF_STEP_BEGIN = 0,
    MZF_STEP_GAP,
    MZF_STEP_TAPE_MARK_LONG,
    MZF_STEP_TAPE_MARK_SHORT,
    MZF_STEP_TAPE_MARK_FINAL,
    MZF_STEP_BYTE_LOAD,
    MZF_STEP_BYTE_BITS,
    MZF_STEP_BYTE_STOP,
    MZF_STEP_CHECKSUM_LOAD,
    MZF_STEP_CHECKSUM_BITS,
    MZF_STEP_CHECKSUM_STOP,
    MZF_STEP_TRAILING_LONGS,
    MZF_STEP_BOUNDARY
} mzf_normal_step_t;

static volatile uint8_t mzf_state = MZF_PLAYBACK_STOPPED;
static char mzf_error_text[17];

static file_format_t mzf_format = FILE_FORMAT_UNKNOWN;

/* MZF/MZT/M12 always use the native Sharp CMT polarity. */

static uint8_t mzf_header[MZF_HEADER_BYTES];
static uint8_t mzf_header_offset = 0U;

static volatile uint16_t mzf_fifo_read_sequence = 0U;
static volatile uint16_t mzf_fifo_write_sequence = 0U;
static volatile uint8_t mzf_fifo_source_finished = 1U;

static uint32_t mzf_file_size = 0UL;
static uint32_t mzf_record_data_length = 0UL;
/* Absolute end position of the current declared data record. */
static uint32_t mzf_record_data_file_end = 0UL;
static uint32_t mzf_record_data_read = 0UL;
/* Calculated in foreground before playback; no progress counters are kept in ISR. */
static uint32_t mzf_total_duration_ms = 0UL;

static mzf_stage_t mzf_stage = MZF_STAGE_NONE;
static mzf_normal_step_t mzf_normal_step = MZF_STEP_BEGIN;
static uint32_t mzf_normal_loop = 0UL;
static uint32_t mzf_normal_bytes_remaining = 0UL;
static uint16_t mzf_normal_checksum = 0U;
static uint8_t mzf_normal_data = 0U;
static uint8_t mzf_normal_bits_remaining = 0U;
static uint8_t mzf_normal_checksum_byte_index = 0U;
static bool mzf_normal_header_preamble = true;

/* Written by Timer3 ISR and read in foreground. */
static volatile bool mzf_boundary_waiting = false;
static volatile uint8_t mzf_motor_low_seen = 0U;
static bool mzf_timer_phase_high = false;
static bool mzf_current_pulse_is_long = false;
static bool mzf_paused_mid_pulse = false;
static uint16_t mzf_paused_remaining_ticks = 0U;

/* Foreground-only fallback timer for a missing/short MOTOR boundary. */
static bool mzf_boundary_auto_timer_armed = false;
static uint32_t mzf_boundary_auto_start_ms = 0UL;

static void mzf_set_error_P(PGM_P text, mzf_playback_state_t state)
{
    flash_text_copy(mzf_error_text, sizeof(mzf_error_text), text);
    mzf_state = (uint8_t)state;
    mz_sense_set(true);
}

static void mzf_set_error_from_isr_P(PGM_P text, mzf_playback_state_t state)
{
    flash_text_copy(mzf_error_text, sizeof(mzf_error_text), text);
    mzf_state = (uint8_t)state;
    mz_read_set_fast_from_isr(0U);
    mz_sense_set_fast(true);
}

static void mzf_stop_timer_from_isr(void)
{
    TIMSK3 &= (uint8_t)~_BV(OCIE3B);
    TCCR3A = 0U;
    TCCR3B = 0U;
    TIFR3 = _BV(OCF3B);
    if (timer3b_owner_get_from_isr() == TIMER3B_OWNER_MZF)
    {
        timer3b_owner_set_from_isr(TIMER3B_OWNER_NONE);
    }
}

static void mzf_stop_timer_from_foreground(bool force_low)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TIMSK3 &= (uint8_t)~_BV(OCIE3B);
        TCCR3A = 0U;
        TCCR3B = 0U;
        TIFR3 = _BV(OCF3B);
        if (timer3b_owner_get_from_isr() == TIMER3B_OWNER_MZF)
        {
            timer3b_owner_set_from_isr(TIMER3B_OWNER_NONE);
        }
    }
    if (force_low)
    {
        mz_read_set(false);
    }
}

static void mzf_timer_start_first(uint16_t ticks)
{
    if (ticks == 0U) ticks = 1U;
    TCNT3 = 0U;
    TCCR3A = 0U;
    TCCR3B = _BV(CS30);
    TIFR3 = _BV(OCF3B);
    OCR3B = ticks;
    timer3b_owner_set_from_isr(TIMER3B_OWNER_MZF);
    TIMSK3 |= _BV(OCIE3B);
}

static void mzf_timer_start_resume(uint16_t ticks)
{
    mzf_timer_start_first(ticks);
}

static uint16_t mzf_fifo_used_snapshot(void)
{
    uint16_t read_sequence;
    uint16_t write_sequence;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        read_sequence = mzf_fifo_read_sequence;
        write_sequence = mzf_fifo_write_sequence;
    }
    return (uint16_t)(write_sequence - read_sequence);
}

static void mzf_fifo_reset(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        mzf_fifo_read_sequence = 0U;
        mzf_fifo_write_sequence = 0U;
        mzf_fifo_source_finished = 0U;
    }
}

static bool mzf_fifo_pop_from_isr(uint8_t *value)
{
    uint16_t read_sequence = mzf_fifo_read_sequence;

    if ((value == NULL) || (read_sequence == mzf_fifo_write_sequence))
    {
        return false;
    }

    *value = wav_sample_stream_isr_bytes[read_sequence & MZF_FIFO_MASK];
    mzf_fifo_read_sequence = (uint16_t)(read_sequence + 1U);
    return true;
}

static bool mzf_refill_data_once(void)
{
    uint16_t used;
    uint16_t free_bytes;
    uint16_t request;
    int16_t received;
    uint16_t write_sequence;
    uint8_t *work;

    if (mzf_record_data_read >= mzf_record_data_length)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            mzf_fifo_source_finished = 1U;
        }
        return true;
    }

    used = mzf_fifo_used_snapshot();
    if (used > MZF_FIFO_CAPACITY)
    {
        mzf_set_error_P(PSTR("MZF FIFO"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }

    free_bytes = (uint16_t)(MZF_FIFO_CAPACITY - used);
    if (free_bytes == 0U)
    {
        return true;
    }

    request = free_bytes;
    if (request > MZF_REFILL_BLOCK)
    {
        request = MZF_REFILL_BLOCK;
    }
    if ((uint32_t)request > (mzf_record_data_length - mzf_record_data_read))
    {
        request = (uint16_t)(mzf_record_data_length - mzf_record_data_read);
    }

    work = wav_sample_stream_get_shared_work_buffer();
    received = sdcard_file_read(work, request);
    if (received < 0)
    {
        mzf_set_error_P(PSTR("MZF READ"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }
    if (received == 0)
    {
        mzf_set_error_P(PSTR("MZF SHORT"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    write_sequence = mzf_fifo_write_sequence;
    for (int16_t i = 0; i < received; ++i)
    {
        wav_sample_stream_isr_bytes[write_sequence & MZF_FIFO_MASK] = work[i];
        write_sequence = (uint16_t)(write_sequence + 1U);
    }

    mzf_record_data_read += (uint32_t)received;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        asm volatile("" ::: "memory");
        mzf_fifo_write_sequence = write_sequence;
        if (mzf_record_data_read >= mzf_record_data_length)
        {
            mzf_fifo_source_finished = 1U;
        }
    }
    return true;
}

static bool mzf_prefill_data(void)
{
    while (mzf_record_data_read < mzf_record_data_length)
    {
        uint16_t before = mzf_fifo_used_snapshot();
        if (!mzf_refill_data_once())
        {
            return false;
        }
        if (mzf_fifo_used_snapshot() == before)
        {
            break;
        }
    }
    return (mzf_record_data_length == 0UL) || (mzf_fifo_used_snapshot() != 0U);
}

static uint32_t mzf_header_data_length(void)
{
    return (uint32_t)mzf_header[MZF_HEADER_DATA_LENGTH_OFFSET] |
           ((uint32_t)mzf_header[MZF_HEADER_DATA_LENGTH_OFFSET + 1U] << 8U);
}

static bool mzf_read_header_record(void)
{
    int16_t received;
    uint32_t remaining;

    if ((mzf_file_size - sdcard_file_position()) < MZF_HEADER_BYTES)
    {
        mzf_set_error_P(PSTR("MZT HEADER"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    received = sdcard_file_read(mzf_header, MZF_HEADER_BYTES);
    if (received != (int16_t)MZF_HEADER_BYTES)
    {
        if (received < 0)
        {
            mzf_set_error_P(PSTR("MZT READ"), MZF_PLAYBACK_IO_ERROR);
        }
        else
        {
            mzf_set_error_P(PSTR("MZT SHORT"), MZF_PLAYBACK_BAD_FILE);
        }
        return false;
    }

    mzf_record_data_length = mzf_header_data_length();
    remaining = mzf_file_size - sdcard_file_position();
    if (mzf_record_data_length > remaining)
    {
        mzf_set_error_P(PSTR("MZF LENGTH"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    mzf_record_data_file_end = sdcard_file_position() + mzf_record_data_length;
    mzf_header_offset = 0U;
    mzf_record_data_read = 0UL;
    mzf_fifo_reset();
    return true;
}

static uint8_t mzf_popcount8(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static uint8_t mzf_popcount16(uint16_t value)
{
    return (uint8_t)(mzf_popcount8((uint8_t)value) +
                     mzf_popcount8((uint8_t)(value >> 8U)));
}

static bool mzf_add_half_milliseconds(uint32_t *total, uint32_t amount)
{
    if (total == NULL) return false;
    if ((0xFFFFFFFFUL - *total) < amount)
    {
        *total = 0xFFFFFFFFUL;
        return false;
    }
    *total += amount;
    return true;
}

/* Every short pulse is 0.5 ms and every long pulse is 1 ms. This computes
   one header/data monitor frame exactly from its byte count and number of
   one bits. The checksum is the same wrapping one-bit count used by the ISR. */
static bool mzf_add_stage_duration(bool header_stage,
                                   uint32_t byte_count,
                                   uint32_t one_count,
                                   uint16_t checksum,
                                   uint32_t *half_milliseconds)
{
    uint32_t preamble = header_stage ? 6524UL : 6464UL;
    uint32_t byte_time;
    uint32_t checksum_and_trailer;

    if (byte_count > 429496729UL)
    {
        *half_milliseconds = 0xFFFFFFFFUL;
        return false;
    }
    byte_time = byte_count * 10UL;
    if ((0xFFFFFFFFUL - byte_time) < one_count)
    {
        *half_milliseconds = 0xFFFFFFFFUL;
        return false;
    }
    byte_time += one_count;
    checksum_and_trailer = 24UL + (uint32_t)mzf_popcount16(checksum);

    return mzf_add_half_milliseconds(half_milliseconds, preamble) &&
           mzf_add_half_milliseconds(half_milliseconds, byte_time) &&
           mzf_add_half_milliseconds(half_milliseconds, checksum_and_trailer);
}

static bool mzf_scan_payload_ones(uint32_t length,
                                  uint32_t *one_count,
                                  uint16_t *checksum)
{
    uint8_t *work = wav_sample_stream_get_shared_work_buffer();

    if ((one_count == NULL) || (checksum == NULL)) return false;
    *one_count = 0UL;
    *checksum = 0U;

    while (length != 0UL)
    {
        uint16_t request = (length > MZF_REFILL_BLOCK) ?
            MZF_REFILL_BLOCK : (uint16_t)length;
        int16_t received = sdcard_file_read(work, request);

        if (received != (int16_t)request)
        {
            mzf_set_error_P((received < 0) ? PSTR("MZF READ") : PSTR("MZF SHORT"),
                            (received < 0) ? MZF_PLAYBACK_IO_ERROR : MZF_PLAYBACK_BAD_FILE);
            return false;
        }

        for (uint16_t index = 0U; index < request; ++index)
        {
            uint8_t ones = mzf_popcount8(work[index]);
            if ((0xFFFFFFFFUL - *one_count) < (uint32_t)ones)
            {
                *one_count = 0xFFFFFFFFUL;
            }
            else
            {
                *one_count += (uint32_t)ones;
            }
            *checksum = (uint16_t)(*checksum + (uint16_t)ones);
        }
        length -= (uint32_t)request;
    }
    return true;
}

static bool mzf_scan_header_record_duration(uint32_t *half_milliseconds)
{
    uint32_t remaining;
    uint32_t header_ones = 0UL;
    uint16_t header_checksum = 0U;
    uint32_t data_ones;
    uint16_t data_checksum;
    int16_t received;

    if ((mzf_file_size - sdcard_file_position()) < MZF_HEADER_BYTES)
    {
        mzf_set_error_P(PSTR("MZT HEADER"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    received = sdcard_file_read(mzf_header, MZF_HEADER_BYTES);
    if (received != (int16_t)MZF_HEADER_BYTES)
    {
        mzf_set_error_P((received < 0) ? PSTR("MZT READ") : PSTR("MZT SHORT"),
                        (received < 0) ? MZF_PLAYBACK_IO_ERROR : MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    for (uint8_t index = 0U; index < MZF_HEADER_BYTES; ++index)
    {
        uint8_t ones = mzf_popcount8(mzf_header[index]);
        header_ones += (uint32_t)ones;
        header_checksum = (uint16_t)(header_checksum + (uint16_t)ones);
    }

    mzf_record_data_length = mzf_header_data_length();
    remaining = mzf_file_size - sdcard_file_position();
    if (mzf_record_data_length > remaining)
    {
        mzf_set_error_P(PSTR("MZF LENGTH"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    if (!mzf_add_stage_duration(true, MZF_HEADER_BYTES, header_ones,
                                header_checksum, half_milliseconds) ||
        !mzf_scan_payload_ones(mzf_record_data_length, &data_ones, &data_checksum) ||
        !mzf_add_stage_duration(false, mzf_record_data_length, data_ones,
                                data_checksum, half_milliseconds))
    {
        return false;
    }
    return true;
}

static bool mzf_calculate_total_duration(void)
{
    uint32_t half_milliseconds = 0UL;

    mzf_total_duration_ms = 0UL;
    if (!sdcard_file_seek(0UL))
    {
        mzf_set_error_P(PSTR("MZF SEEK"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }

    if (mzf_format == FILE_FORMAT_MZT)
    {
        do
        {
            if (!mzf_scan_header_record_duration(&half_milliseconds)) return false;
        }
        while (sdcard_file_position() < mzf_file_size);
    }
    else
    {
        uint32_t trailing_length;
        uint32_t trailing_ones;
        uint16_t trailing_checksum;

        if (!mzf_scan_header_record_duration(&half_milliseconds)) return false;
        trailing_length = mzf_file_size - sdcard_file_position();
        if (trailing_length != 0UL)
        {
            if (!mzf_scan_payload_ones(trailing_length, &trailing_ones,
                                       &trailing_checksum) ||
                !mzf_add_stage_duration(false, trailing_length, trailing_ones,
                                        trailing_checksum, &half_milliseconds))
            {
                return false;
            }
        }
    }

    mzf_total_duration_ms = (half_milliseconds == 0xFFFFFFFFUL) ?
        0xFFFFFFFFUL : (half_milliseconds + 1UL) / 2UL;

    if (!sdcard_file_seek(0UL))
    {
        mzf_set_error_P(PSTR("MZF SEEK"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }
    return true;
}


static void mzf_begin_normal_stage(mzf_stage_t stage)
{
    mzf_stage = stage;
    mzf_normal_step = MZF_STEP_BEGIN;
    mzf_normal_loop = 0UL;
    mzf_normal_bytes_remaining = 0UL;
    mzf_normal_checksum = 0U;
    mzf_normal_data = 0U;
    mzf_normal_bits_remaining = 0U;
    mzf_normal_checksum_byte_index = 0U;
    mzf_normal_header_preamble = (stage == MZF_STAGE_HEADER);
    mzf_boundary_waiting = false;
    mzf_motor_low_seen = 0U;
    mzf_boundary_auto_timer_armed = false;
    mzf_boundary_auto_start_ms = 0UL;
    mzf_paused_mid_pulse = false;
    mzf_timer_phase_high = false;
    mzf_current_pulse_is_long = false;

    if (stage == MZF_STAGE_HEADER)
    {
        mzf_header_offset = 0U;
    }
}

static uint32_t mzf_stage_byte_count(void)
{
    if (mzf_stage == MZF_STAGE_HEADER) return MZF_HEADER_BYTES;
    if (mzf_stage == MZF_STAGE_DATA) return mzf_record_data_length;
    return 0UL;
}

static bool mzf_next_source_byte_from_isr(uint8_t *value)
{
    if (value == NULL)
    {
        return false;
    }

    if (mzf_stage == MZF_STAGE_HEADER)
    {
        if (mzf_header_offset >= MZF_HEADER_BYTES)
        {
            return false;
        }
        *value = mzf_header[mzf_header_offset++];
        return true;
    }

    if (mzf_stage == MZF_STAGE_DATA)
    {
        if (!mzf_fifo_pop_from_isr(value))
        {
            return false;
        }
        return true;
    }

    return false;
}

static bool mzf_next_normal_pulse_from_isr(uint16_t *high_ticks,
                                            uint16_t *low_ticks)
{
    uint32_t byte_count;

    if ((high_ticks == NULL) || (low_ticks == NULL))
    {
        return false;
    }

    for (;;)
    {
        switch (mzf_normal_step)
        {
            case MZF_STEP_BEGIN:
                mzf_normal_loop = mzf_normal_header_preamble ?
                    MZF_MZ800_LONG_GAP_SHORT_PULSES :
                    MZF_MZ800_SHORT_GAP_SHORT_PULSES;
                mzf_normal_step = MZF_STEP_GAP;
                continue;

            case MZF_STEP_GAP:
                if (mzf_normal_loop == 0UL)
                {
                    mzf_normal_loop = mzf_normal_header_preamble ?
                        MZF_MZ800_LONG_MARK_LONG_PULSES :
                        MZF_MZ800_SHORT_MARK_LONG_PULSES;
                    mzf_normal_step = MZF_STEP_TAPE_MARK_LONG;
                    continue;
                }
                *high_ticks = MZF_SHORT_HIGH_TICKS;
                *low_ticks = MZF_SHORT_LOW_TICKS;
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_LONG:
                if (mzf_normal_loop == 0UL)
                {
                    mzf_normal_loop = mzf_normal_header_preamble ?
                        MZF_MZ800_LONG_MARK_SHORT_PULSES :
                        MZF_MZ800_SHORT_MARK_SHORT_PULSES;
                    mzf_normal_step = MZF_STEP_TAPE_MARK_SHORT;
                    continue;
                }
                *high_ticks = MZF_LONG_HIGH_TICKS;
                *low_ticks = MZF_LONG_LOW_TICKS;
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_SHORT:
                if (mzf_normal_loop == 0UL)
                {
                    mzf_normal_loop = MZF_MZ800_TAPE_MARK_FINAL_LONG_PULSES;
                    mzf_normal_step = MZF_STEP_TAPE_MARK_FINAL;
                    continue;
                }
                *high_ticks = MZF_SHORT_HIGH_TICKS;
                *low_ticks = MZF_SHORT_LOW_TICKS;
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_FINAL:
                if (mzf_normal_loop == 0UL)
                {
                    byte_count = mzf_stage_byte_count();
                    mzf_normal_bytes_remaining = byte_count;
                    mzf_normal_checksum = 0U;
                    mzf_normal_checksum_byte_index = 0U;
                    mzf_normal_step = MZF_STEP_BYTE_LOAD;
                    continue;
                }
                *high_ticks = MZF_LONG_HIGH_TICKS;
                *low_ticks = MZF_LONG_LOW_TICKS;
                mzf_normal_loop--;
                return true;

            case MZF_STEP_BYTE_LOAD:
                if (mzf_normal_bytes_remaining == 0UL)
                {
                    mzf_normal_checksum_byte_index = 0U;
                    mzf_normal_step = MZF_STEP_CHECKSUM_LOAD;
                    continue;
                }
                if (!mzf_next_source_byte_from_isr(&mzf_normal_data))
                {
                    mzf_set_error_from_isr_P(PSTR("MZF UNDER"), MZF_PLAYBACK_UNDERRUN);
                    return false;
                }
                mzf_normal_bytes_remaining--;
                mzf_normal_bits_remaining = 8U;
                mzf_normal_step = MZF_STEP_BYTE_BITS;
                continue;

            case MZF_STEP_BYTE_BITS:
                if (mzf_normal_bits_remaining == 0U)
                {
                    mzf_normal_step = MZF_STEP_BYTE_STOP;
                    continue;
                }
                if ((mzf_normal_data & 0x80U) != 0U)
                {
                    *high_ticks = MZF_LONG_HIGH_TICKS;
                    *low_ticks = MZF_LONG_LOW_TICKS;
                    mzf_normal_checksum++;
                }
                else
                {
                    *high_ticks = MZF_SHORT_HIGH_TICKS;
                    *low_ticks = MZF_SHORT_LOW_TICKS;
                }
                mzf_normal_data <<= 1U;
                mzf_normal_bits_remaining--;
                return true;

            case MZF_STEP_BYTE_STOP:
                /* MZ-800 ROM format: one long stop pulse follows every byte. */
                *high_ticks = MZF_LONG_HIGH_TICKS;
                *low_ticks = MZF_LONG_LOW_TICKS;
                mzf_normal_step = MZF_STEP_BYTE_LOAD;
                return true;

            case MZF_STEP_CHECKSUM_LOAD:
                if (mzf_normal_checksum_byte_index >= 2U)
                {
                    mzf_normal_loop = MZF_MZ800_TRAILING_LONG_PULSES;
                    mzf_normal_step = MZF_STEP_TRAILING_LONGS;
                    continue;
                }
                mzf_normal_data = (mzf_normal_checksum_byte_index == 0U) ?
                    (uint8_t)(mzf_normal_checksum >> 8U) :
                    (uint8_t)(mzf_normal_checksum & 0xFFU);
                mzf_normal_checksum_byte_index++;
                mzf_normal_bits_remaining = 8U;
                mzf_normal_step = MZF_STEP_CHECKSUM_BITS;
                continue;

            case MZF_STEP_CHECKSUM_BITS:
                if (mzf_normal_bits_remaining == 0U)
                {
                    mzf_normal_step = MZF_STEP_CHECKSUM_STOP;
                    continue;
                }
                if ((mzf_normal_data & 0x80U) != 0U)
                {
                    *high_ticks = MZF_LONG_HIGH_TICKS;
                    *low_ticks = MZF_LONG_LOW_TICKS;
                }
                else
                {
                    *high_ticks = MZF_SHORT_HIGH_TICKS;
                    *low_ticks = MZF_SHORT_LOW_TICKS;
                }
                mzf_normal_data <<= 1U;
                mzf_normal_bits_remaining--;
                return true;

            case MZF_STEP_CHECKSUM_STOP:
                *high_ticks = MZF_LONG_HIGH_TICKS;
                *low_ticks = MZF_LONG_LOW_TICKS;
                mzf_normal_step = MZF_STEP_CHECKSUM_LOAD;
                return true;

            case MZF_STEP_TRAILING_LONGS:
                if (mzf_normal_loop == 0UL)
                {
                    mzf_normal_step = MZF_STEP_BOUNDARY;
                    continue;
                }
                *high_ticks = MZF_LONG_HIGH_TICKS;
                *low_ticks = MZF_LONG_LOW_TICKS;
                mzf_normal_loop--;
                return true;

            case MZF_STEP_BOUNDARY:
                /*
                    A final declared data record does not require another
                    MOTOR edge. Finish here so a single-record MZF/M12 and
                    the last MZT record return to the browser immediately.
                    Earlier records remain at the boundary and advance only
                    after the monitor turns MOTOR off.
                */
                if ((mzf_stage == MZF_STAGE_DATA) &&
                    (mzf_record_data_file_end >= mzf_file_size))
                {
                    mzf_stop_timer_from_isr();
                    mz_read_set_fast_from_isr(0U);
                    mz_sense_set_fast(true);
                    mzf_state = MZF_PLAYBACK_FINISHED;
                    return false;
                }

                mzf_boundary_waiting = true;
                mzf_stop_timer_from_isr();
                mz_read_set_fast_from_isr(0U);
                return false;

            default:
                mzf_set_error_from_isr_P(PSTR("MZF STATE"), MZF_PLAYBACK_BAD_FILE);
                return false;
        }
    }
}

static bool mzf_start_next_normal_pulse_from_isr(void)
{
    uint16_t high_ticks;
    uint16_t low_ticks;

    if (!mzf_next_normal_pulse_from_isr(&high_ticks, &low_ticks))
    {
        return false;
    }

    mzf_timer_phase_high = true;
    mzf_current_pulse_is_long = (high_ticks == MZF_LONG_HIGH_TICKS);
    mz_read_set_fast_from_isr(1U);
    (void)low_ticks;
    mzf_paused_remaining_ticks = high_ticks;
    mzf_timer_start_first(high_ticks);
    return true;
}

/* Low ticks are determined from the current high/short-or-long pulse class. */
static uint16_t mzf_current_low_ticks(void)
{
    return mzf_current_pulse_is_long ?
        MZF_LONG_LOW_TICKS : MZF_SHORT_LOW_TICKS;
}

static bool mzf_start_normal_output(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        mzf_start_next_normal_pulse_from_isr();
    }
    return (mzf_state == MZF_PLAYBACK_RUNNING) && !mzf_boundary_waiting;
}

static bool mzf_start_next_mzt_record(void)
{
    if (sdcard_file_position() >= mzf_file_size)
    {
        return false;
    }

    if (!mzf_read_header_record())
    {
        return false;
    }
    if (!mzf_prefill_data())
    {
        return false;
    }

    mzf_begin_normal_stage(MZF_STAGE_HEADER);
    return true;
}

static bool mzf_advance_after_boundary(void)
{
    if (mzf_stage == MZF_STAGE_HEADER)
    {
        mzf_begin_normal_stage(MZF_STAGE_DATA);
        return true;
    }

    if (mzf_stage != MZF_STAGE_DATA)
    {
        mzf_set_error_P(PSTR("MZF STATE"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    if (mzf_format == FILE_FORMAT_MZT)
    {
        if (sdcard_file_position() < mzf_file_size)
        {
            return mzf_start_next_mzt_record();
        }
    }
    else if (sdcard_file_position() < mzf_file_size)
    {
        /* Preserve legacy MZF/M12 compatibility: trailing bytes are emitted
           as a follow-on data block after the next MOTOR restart. */
        mzf_record_data_length = mzf_file_size - sdcard_file_position();
        mzf_record_data_file_end = mzf_file_size;
        mzf_record_data_read = 0UL;
        mzf_fifo_reset();
        if (!mzf_prefill_data())
        {
            return false;
        }
        mzf_begin_normal_stage(MZF_STAGE_DATA);
        return true;
    }

    mzf_state = MZF_PLAYBACK_FINISHED;
    mz_sense_set(true);
    return true;
}

/*
    Continue a completed PWM block when MOTOR stays high.  This is required
    in PLAY CTRL / MANUAL and is also a safe fallback when the MZ supplies a
    short MOTOR-low indication which the foreground control loop cannot use
    as a persistent pause.  A real MOTOR-low level still follows the legacy
    pause/resume path through mzf_playback_pause().
*/
static void mzf_service_boundary_auto_continue(void)
{
    uint32_t now;

    if (!mzf_boundary_waiting || (mzf_state != MZF_PLAYBACK_RUNNING))
    {
        return;
    }

    /* Let the controller preserve the native MOTOR-low pause behavior. */
    if (!mz_motor_get())
    {
        return;
    }

    now = millis();
    if (!mzf_boundary_auto_timer_armed)
    {
        mzf_boundary_auto_start_ms = now;
        mzf_boundary_auto_timer_armed = true;

        /* A low edge observed at timer precision has already completed the
           legacy boundary; do not add an unnecessary silent delay. */
        if (mzf_motor_low_seen == 0U)
        {
            return;
        }
    }
    else if ((mzf_motor_low_seen == 0U) &&
             ((uint32_t)(now - mzf_boundary_auto_start_ms) <
              MZF_BOUNDARY_AUTO_CONTINUE_MS))
    {
        return;
    }

    if (!mzf_advance_after_boundary())
    {
        return;
    }

    if (mzf_state == MZF_PLAYBACK_FINISHED)
    {
        return;
    }

    if (!mzf_start_normal_output() && (mzf_state == MZF_PLAYBACK_RUNNING))
    {
        mzf_set_error_P(PSTR("MZF START"), MZF_PLAYBACK_BAD_FILE);
    }
}

void mzf_playback_init(void)
{
    mzf_stop_timer_from_foreground(true);
    mzf_state = MZF_PLAYBACK_STOPPED;
    mzf_error_text[0] = '\0';
    mzf_format = FILE_FORMAT_UNKNOWN;
    mzf_file_size = 0UL;
    mzf_total_duration_ms = 0UL;
    mzf_record_data_length = 0UL;
    mzf_record_data_file_end = 0UL;
    mzf_record_data_read = 0UL;
    mzf_stage = MZF_STAGE_NONE;
    mzf_boundary_waiting = false;
    mzf_motor_low_seen = 0U;
    mzf_boundary_auto_timer_armed = false;
    mzf_boundary_auto_start_ms = 0UL;
    mzf_timer_phase_high = false;
    mzf_current_pulse_is_long = false;
    mzf_paused_mid_pulse = false;
    mzf_paused_remaining_ticks = 0U;
    mzf_fifo_reset();
}

bool mzf_playback_prepare(const char *path,
                          file_format_t format)
{
    mzf_playback_stop();
    mzf_error_text[0] = '\0';

    if ((path == NULL) || !file_format_is_sharp_tape(format))
    {
        mzf_set_error_P(PSTR("MZF ARG"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    mzf_format = format;

    if (!sdcard_file_open_read(path))
    {
        mzf_set_error_P(PSTR("MZF OPEN"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }

    mzf_file_size = sdcard_file_size();
    if (mzf_file_size < MZF_HEADER_BYTES)
    {
        sdcard_file_close();
        mzf_set_error_P(PSTR("MZF SHORT"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    if (!mzf_calculate_total_duration() || !mzf_read_header_record())
    {
        sdcard_file_close();
        return false;
    }

    if (!mzf_prefill_data())
    {
        sdcard_file_close();
        return false;
    }

    mzf_begin_normal_stage(MZF_STAGE_HEADER);
    mzf_state = MZF_PLAYBACK_READY;
    return true;
}

bool mzf_playback_start(void)
{
    if ((mzf_state != MZF_PLAYBACK_READY) && (mzf_state != MZF_PLAYBACK_PAUSED))
    {
        return false;
    }

    mzf_state = MZF_PLAYBACK_RUNNING;
    mz_sense_set(false);

    if (mzf_paused_mid_pulse)
    {
        /* This low MOTOR belonged to a mid-block pause, not to the next
           header/data boundary. */
        mzf_motor_low_seen = 0U;
        mzf_paused_mid_pulse = false;
        mzf_timer_start_resume(mzf_paused_remaining_ticks);
        return true;
    }

    return mzf_start_normal_output();
}

bool mzf_playback_pause(void)
{
    uint16_t remaining;

    if (mzf_state != MZF_PLAYBACK_RUNNING)
    {
        return false;
    }

    if (mzf_boundary_waiting)
    {
        if (!mzf_advance_after_boundary())
        {
            return false;
        }
        if (mzf_state == MZF_PLAYBACK_FINISHED)
        {
            return true;
        }

        mzf_state = MZF_PLAYBACK_PAUSED;
        return true;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        remaining = (uint16_t)(OCR3B - TCNT3);
        if (((TIFR3 & _BV(OCF3B)) != 0U) || (remaining == 0U))
        {
            remaining = 1U;
        }
        mzf_paused_remaining_ticks = remaining;
        mzf_stop_timer_from_isr();
        mzf_paused_mid_pulse = true;
        mzf_state = MZF_PLAYBACK_PAUSED;
    }
    return true;
}

bool mzf_playback_resume(void)
{
    return mzf_playback_start();
}

void mzf_playback_stop(void)
{
    mzf_stop_timer_from_foreground(true);
    sdcard_file_close();
    mzf_fifo_reset();
    mzf_state = MZF_PLAYBACK_STOPPED;
    mzf_stage = MZF_STAGE_NONE;
    mzf_boundary_waiting = false;
    mzf_motor_low_seen = 0U;
    mzf_boundary_auto_timer_armed = false;
    mzf_boundary_auto_start_ms = 0UL;
    mzf_timer_phase_high = false;
    mzf_current_pulse_is_long = false;
    mzf_paused_mid_pulse = false;
    mzf_paused_remaining_ticks = 0U;
    mzf_file_size = 0UL;
    mzf_total_duration_ms = 0UL;
    mzf_record_data_length = 0UL;
    mzf_record_data_file_end = 0UL;
    mzf_record_data_read = 0UL;
    mz_sense_set(true);
}

void mzf_playback_service(void)
{
    if (mzf_state != MZF_PLAYBACK_RUNNING)
    {
        return;
    }

    if (mzf_boundary_waiting)
    {
        mzf_service_boundary_auto_continue();
        return;
    }

    if ((mzf_stage == MZF_STAGE_DATA) && !mzf_boundary_waiting &&
        (mzf_fifo_used_snapshot() <= MZF_REFILL_BLOCK) &&
        (mzf_record_data_read < mzf_record_data_length))
    {
        (void)mzf_refill_data_once();
    }
}

mzf_playback_state_t mzf_playback_get_state(void)
{
    uint8_t state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        state = mzf_state;
    }
    return (mzf_playback_state_t)state;
}

const char *mzf_playback_get_error_text(void) { return mzf_error_text; }

uint8_t mzf_playback_get_buffer_fill_percent(void)
{
    uint32_t percent;

    if (mzf_stage != MZF_STAGE_DATA)
    {
        return 100U;
    }
    if (mzf_record_data_length == 0UL)
    {
        return 100U;
    }

    percent = ((uint32_t)mzf_fifo_used_snapshot() * 100UL) / MZF_FIFO_CAPACITY;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

uint32_t mzf_playback_get_total_duration_ms(void)
{
    return mzf_total_duration_ms;
}

/*
    Called by the sole TIMER3_COMPB_vect dispatcher in edge_playback.cpp.
    Called only while the Timer3B owner is MZF.  The shared dispatcher keeps
    idle MZF state from touching an active LEP/L16 transport.
*/
bool mzf_playback_timer3_compb_from_isr(void)
{
    uint16_t low_ticks;

    if (mzf_state != MZF_PLAYBACK_RUNNING)
    {
        return false;
    }

    /* Sample MOTOR at pulse precision: a short low boundary must not be
       lost between foreground play-controller polls. */
    if (mz_motor_sample_from_isr() == 0U)
    {
        mzf_motor_low_seen = 1U;
    }

    if (mzf_timer_phase_high)
    {
        mzf_timer_phase_high = false;
        mz_read_set_fast_from_isr(0U);
        low_ticks = mzf_current_low_ticks();
        mzf_timer_start_resume(low_ticks);
        return true;
    }

    (void)mzf_start_next_normal_pulse_from_isr();
    return true;
}
