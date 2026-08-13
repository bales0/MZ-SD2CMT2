#include "mzf_playback.h"
#include "timer3b_owner.h"
#include "mzf_loader.h"

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
#define MZF_IC_1_4_SHORT_HIGH_TICKS MZF_US_TO_TICKS(112U)
#define MZF_IC_1_4_SHORT_LOW_TICKS  MZF_US_TO_TICKS(80U)
#define MZF_IC_1_4_LONG_HIGH_TICKS  MZF_US_TO_TICKS(176U)
#define MZF_IC_1_4_LONG_LOW_TICKS   MZF_US_TO_TICKS(160U)
#define MZF_IC_1_3_SHORT_HIGH_TICKS MZF_US_TO_TICKS(112U)
#define MZF_IC_1_3_SHORT_LOW_TICKS  MZF_US_TO_TICKS(96U)
#define MZF_IC_1_3_LONG_HIGH_TICKS  MZF_US_TO_TICKS(224U)
#define MZF_IC_1_3_LONG_LOW_TICKS   MZF_US_TO_TICKS(192U)
/* MZ-700 1Z-009A FAST3 runs the patched DLY3 routine from RAM.  With the
   verified $15 operand its input sample point is about 97 us after the edge.
   Keep both halves symmetric and leave margin for the software-edge ISR:
   short below the sample point, long at exactly twice the short interval. */
#define MZF_MZ700_3X_SHORT_TICKS MZF_US_TO_TICKS(80U)
#define MZF_MZ700_3X_LONG_TICKS  MZF_US_TO_TICKS(160U)
#define MZF_IC_1_2_SHORT_HIGH_TICKS MZF_US_TO_TICKS(144U)
#define MZF_IC_1_2_SHORT_LOW_TICKS  MZF_US_TO_TICKS(112U)
#define MZF_IC_1_2_LONG_HIGH_TICKS  MZF_US_TO_TICKS(256U)
#define MZF_IC_1_2_LONG_LOW_TICKS   MZF_US_TO_TICKS(224U)
#define MZF_TC_1_3_SHORT_HIGH_TICKS MZF_US_TO_TICKS(112U)
#define MZF_TC_1_3_SHORT_LOW_TICKS  MZF_US_TO_TICKS(112U)
#define MZF_TC_1_3_LONG_HIGH_TICKS  MZF_US_TO_TICKS(204U)
#define MZF_TC_1_3_LONG_LOW_TICKS   MZF_US_TO_TICKS(204U)
#define MZF_TC_1_2_SHORT_HIGH_TICKS MZF_US_TO_TICKS(144U)
#define MZF_TC_1_2_SHORT_LOW_TICKS  MZF_US_TO_TICKS(144U)
#define MZF_TC_1_2_LONG_HIGH_TICKS  MZF_US_TO_TICKS(288U)
#define MZF_TC_1_2_LONG_LOW_TICKS   MZF_US_TO_TICKS(288U)

#define MZF_MZ800_LONG_GAP_SHORT_PULSES 6400U
#define MZF_MZ800_SHORT_GAP_SHORT_PULSES 6400U
#define MZF_MZ800_LONG_MARK_LONG_PULSES 40U
#define MZF_MZ800_LONG_MARK_SHORT_PULSES 40U
#define MZF_MZ800_SHORT_MARK_LONG_PULSES 20U
#define MZF_MZ800_SHORT_MARK_SHORT_PULSES 20U
#define MZF_MZ800_TAPE_MARK_FINAL_LONG_PULSES 2U
#define MZF_MZ800_TRAILING_LONG_PULSES 2U
#define MZF_TC_LOADER_TRAILING_SHORT_PULSES 98U

#define MZF_FIFO_BYTES WAV_SAMPLE_STREAM_BUFFER_BYTES
#define MZF_FIFO_CAPACITY (MZF_FIFO_BYTES - 1U)
#define MZF_FIFO_MASK (MZF_FIFO_BYTES - 1U)
#define MZF_REFILL_BLOCK WAV_SAMPLE_STREAM_REFILL_BLOCK
/*
    Header/data boundaries normally use an MZ MOTOR off/on cycle.  Some
    transports, and PLAY CTRL / MANUAL, leave MOTOR high; do not strand a
    binary image after its 128-byte header in that case.
*/
#define MZF_BOUNDARY_AUTO_CONTINUE_MS 120U
#define MZF_IC_TURBO_START_DELAY_MS 345U
#define MZF_MZ700_FAST3_START_DELAY_MS 400U
#define MZF_TC_TURBO_START_DELAY_MS 110U
#define MZF_IC_TURBO_GAP_SHORT_PULSES 5500U
#define MZF_TC_1_3_TURBO_GAP_SHORT_PULSES 15130U
#define MZF_TC_1_2_TURBO_GAP_SHORT_PULSES 11239U
#define MZF_IC_TURBO_MARK_LONG_PULSES 20U
#define MZF_IC_TURBO_MARK_SHORT_PULSES 20U
#define MZF_IC_TURBO_MARK_FINAL_LONG_PULSES 2U

#if ((MZF_FIFO_BYTES & (MZF_FIFO_BYTES - 1U)) != 0U)
#error "MZF FIFO needs a power-of-two size"
#endif

typedef enum
{
    MZF_STAGE_NONE = 0,
    MZF_STAGE_HEADER,
    MZF_STAGE_DATA,
    MZF_STAGE_TAPE_TURBO_DATA,
    MZF_STAGE_ULTRAFAST
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

/* MZF ignores the UI WAV invert setting. TC turbo is auto-inverted to match
   the verified physical READ phase; IC and native/UL modes stay direct. */
static bool mzf_wave_invert_signal = false;

static uint8_t mzf_header[MZF_HEADER_BYTES];
static volatile uint8_t mzf_header_offset = 0U;

static volatile uint16_t mzf_fifo_read_sequence = 0U;
static volatile uint16_t mzf_fifo_write_sequence = 0U;
static volatile uint8_t mzf_fifo_source_finished = 1U;

static uint32_t mzf_file_size = 0UL;
static uint32_t mzf_record_data_length = 0UL;
/* Absolute end position of the current declared data record. */
static uint32_t mzf_record_data_file_end = 0UL;
static uint32_t mzf_record_data_read = 0UL;
static uint32_t mzf_original_data_offset = 0UL;
static uint32_t mzf_original_data_length = 0UL;
/* Calculated in foreground before playback; no progress counters are kept in ISR. */
static uint32_t mzf_total_duration_ms = 0UL;

static mzf_stage_t mzf_stage = MZF_STAGE_NONE;
static mzf_normal_step_t mzf_normal_step = MZF_STEP_BEGIN;
static uint16_t mzf_normal_loop = 0U;
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
static bool mzf_timer_phase_low_first = false;
static bool mzf_current_pulse_is_long = false;
static bool mzf_paused_mid_pulse = false;
static uint16_t mzf_paused_remaining_ticks = 0U;

/* Foreground-only fallback timer for a missing/short MOTOR boundary. */
static bool mzf_boundary_auto_timer_armed = false;
static uint16_t mzf_boundary_auto_start_ms = 0U;

static void mzf_set_error_P(PGM_P text, mzf_playback_state_t state)
{
    flash_text_copy(mzf_error_text, sizeof(mzf_error_text), text);
    mzf_state = (uint8_t)state;
}

static void mzf_set_error(const char *text, mzf_playback_state_t state)
{
    if (text == NULL)
    {
        flash_text_copy(mzf_error_text, sizeof(mzf_error_text), PSTR("MZF ERROR"));
    }
    else
    {
        strncpy(mzf_error_text, text, sizeof(mzf_error_text) - 1U);
        mzf_error_text[sizeof(mzf_error_text) - 1U] = '\0';
    }
    mzf_state = (uint8_t)state;
}

static void mzf_set_error_from_isr_P(PGM_P text, mzf_playback_state_t state)
{
    flash_text_copy(mzf_error_text, sizeof(mzf_error_text), text);
    mzf_state = (uint8_t)state;
    mz_read_set_fast_from_isr(0U);
}

static inline void mzf_read_wave_set_from_isr(uint8_t level)
{
    mz_read_set_fast_from_isr(level ^ (mzf_wave_invert_signal ? 1U : 0U));
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

/* MZ-700 FAST3 has only about 17 us between the nominal 80 us short phase
   and the patched ROM sample point.  Restarting Timer3 after every READ edge
   adds the ISR body to every phase.  Once the first phase is running, anchor
   subsequent compares to the previous compare point so the ISR latency does
   not accumulate into the generated pulse width. */
static void mzf_timer_start_phase_from_isr(uint16_t ticks)
{
    if (ticks == 0U) ticks = 1U;

    if (mzf_loader_is_mz700_fast3() &&
        (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA) &&
        (timer3b_owner_get_from_isr() == TIMER3B_OWNER_MZF))
    {
        OCR3B = (uint16_t)(OCR3B + ticks);
        return;
    }

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

static bool mzf_prepare_loader_block_data(void)
{
    uint8_t *work = wav_sample_stream_get_shared_work_buffer();
    uint16_t length = mzf_loader_build_loader(work, MZF_REFILL_BLOCK);
    uint16_t write_sequence = 0U;

    if ((length == 0U) || (length > MZF_FIFO_CAPACITY))
    {
        mzf_set_error_P(PSTR("LDR BLOCK"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        wav_sample_stream_isr_bytes[write_sequence & MZF_FIFO_MASK] = work[i];
        write_sequence = (uint16_t)(write_sequence + 1U);
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        mzf_fifo_read_sequence = 0U;
        mzf_fifo_write_sequence = write_sequence;
        mzf_fifo_source_finished = 1U;
    }

    mzf_record_data_length = length;
    mzf_record_data_read = length;
    mzf_record_data_file_end = 0UL;
    return true;
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


static bool mzf_stage_uses_tape_turbo_timing(void)
{
    return mzf_stage == MZF_STAGE_TAPE_TURBO_DATA;
}

static bool mzf_stage_uses_low_first_timing(void)
{
    /* Existing MZ-800 IC loaders use their verified LOW-first phase.
       The hardware-proven MZ-700 FAST3 WAV starts every pulse HIGH-first. */
    return mzf_stage_uses_tape_turbo_timing() && mzf_loader_is_ic_turbo();
}

static uint16_t mzf_boundary_auto_continue_ms(void)
{
    if (((mzf_stage == MZF_STAGE_HEADER) &&
         (mzf_loader_is_ic_turbo() || mzf_loader_is_mz700_fast3())) ||
        ((mzf_stage == MZF_STAGE_DATA) && mzf_loader_is_tape_turbo()))
    {
        if ((mzf_stage == MZF_STAGE_DATA) && mzf_loader_is_tc_turbo())
        {
            return MZF_TC_TURBO_START_DELAY_MS;
        }
        return MZF_IC_TURBO_START_DELAY_MS;
    }
    return MZF_BOUNDARY_AUTO_CONTINUE_MS;
}

static uint16_t mzf_short_high_ticks(void)
{
    if (mzf_stage_uses_tape_turbo_timing())
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_IC_1_2: return MZF_IC_1_2_SHORT_HIGH_TICKS;
            case MZF_LOADER_VARIANT_IC_1_3: return MZF_IC_1_3_SHORT_HIGH_TICKS;
            case MZF_LOADER_VARIANT_MZ700_FAST3_LOW:
            case MZF_LOADER_VARIANT_MZ700_FAST3_HIGH:
                return MZF_MZ700_3X_SHORT_TICKS;
            case MZF_LOADER_VARIANT_TC_1_2: return MZF_TC_1_2_SHORT_HIGH_TICKS;
            case MZF_LOADER_VARIANT_TC_1_3: return MZF_TC_1_3_SHORT_HIGH_TICKS;
            default: return MZF_IC_1_4_SHORT_HIGH_TICKS;
        }
    }
    return MZF_SHORT_HIGH_TICKS;
}

static uint16_t mzf_short_low_ticks(void)
{
    if (mzf_stage_uses_tape_turbo_timing())
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_IC_1_2: return MZF_IC_1_2_SHORT_LOW_TICKS;
            case MZF_LOADER_VARIANT_IC_1_3: return MZF_IC_1_3_SHORT_LOW_TICKS;
            case MZF_LOADER_VARIANT_MZ700_FAST3_LOW:
            case MZF_LOADER_VARIANT_MZ700_FAST3_HIGH:
                return MZF_MZ700_3X_SHORT_TICKS;
            case MZF_LOADER_VARIANT_TC_1_2: return MZF_TC_1_2_SHORT_LOW_TICKS;
            case MZF_LOADER_VARIANT_TC_1_3: return MZF_TC_1_3_SHORT_LOW_TICKS;
            default: return MZF_IC_1_4_SHORT_LOW_TICKS;
        }
    }
    return MZF_SHORT_LOW_TICKS;
}

static uint16_t mzf_long_high_ticks(void)
{
    if (mzf_stage_uses_tape_turbo_timing())
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_IC_1_2: return MZF_IC_1_2_LONG_HIGH_TICKS;
            case MZF_LOADER_VARIANT_IC_1_3: return MZF_IC_1_3_LONG_HIGH_TICKS;
            case MZF_LOADER_VARIANT_MZ700_FAST3_LOW:
            case MZF_LOADER_VARIANT_MZ700_FAST3_HIGH:
                return MZF_MZ700_3X_LONG_TICKS;
            case MZF_LOADER_VARIANT_TC_1_2: return MZF_TC_1_2_LONG_HIGH_TICKS;
            case MZF_LOADER_VARIANT_TC_1_3: return MZF_TC_1_3_LONG_HIGH_TICKS;
            default: return MZF_IC_1_4_LONG_HIGH_TICKS;
        }
    }
    return MZF_LONG_HIGH_TICKS;
}

static uint16_t mzf_long_low_ticks(void)
{
    if (mzf_stage_uses_tape_turbo_timing())
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_IC_1_2: return MZF_IC_1_2_LONG_LOW_TICKS;
            case MZF_LOADER_VARIANT_IC_1_3: return MZF_IC_1_3_LONG_LOW_TICKS;
            case MZF_LOADER_VARIANT_MZ700_FAST3_LOW:
            case MZF_LOADER_VARIANT_MZ700_FAST3_HIGH:
                return MZF_MZ700_3X_LONG_TICKS;
            case MZF_LOADER_VARIANT_TC_1_2: return MZF_TC_1_2_LONG_LOW_TICKS;
            case MZF_LOADER_VARIANT_TC_1_3: return MZF_TC_1_3_LONG_LOW_TICKS;
            default: return MZF_IC_1_4_LONG_LOW_TICKS;
        }
    }
    return MZF_LONG_LOW_TICKS;
}

static void mzf_set_short_pulse(uint16_t *high_ticks, uint16_t *low_ticks)
{
    *high_ticks = mzf_short_high_ticks();
    *low_ticks = mzf_short_low_ticks();
}

static void mzf_set_long_pulse(uint16_t *high_ticks, uint16_t *low_ticks)
{
    *high_ticks = mzf_long_high_ticks();
    *low_ticks = mzf_long_low_ticks();
}

static uint16_t mzf_gap_short_pulses(void)
{
    if (mzf_stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_GAP_SHORT_PULSES;
    }
    if (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_TC_1_2:
                return MZF_TC_1_2_TURBO_GAP_SHORT_PULSES;
            case MZF_LOADER_VARIANT_TC_1_3:
                return MZF_TC_1_3_TURBO_GAP_SHORT_PULSES;
            default:
                return MZF_IC_TURBO_GAP_SHORT_PULSES;
        }
    }
    return MZF_MZ800_SHORT_GAP_SHORT_PULSES;
}

static uint16_t mzf_mark_long_pulses(void)
{
    if (mzf_stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_MARK_LONG_PULSES;
    }
    if (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        return MZF_IC_TURBO_MARK_LONG_PULSES;
    }
    return MZF_MZ800_SHORT_MARK_LONG_PULSES;
}

static uint16_t mzf_mark_short_pulses(void)
{
    if (mzf_stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_MARK_SHORT_PULSES;
    }
    if (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        return MZF_IC_TURBO_MARK_SHORT_PULSES;
    }
    return MZF_MZ800_SHORT_MARK_SHORT_PULSES;
}

static uint16_t mzf_mark_final_long_pulses(void)
{
    return (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA) ?
        MZF_IC_TURBO_MARK_FINAL_LONG_PULSES :
        MZF_MZ800_TAPE_MARK_FINAL_LONG_PULSES;
}

static bool mzf_stage_uses_tc_trailing(void)
{
    return mzf_loader_is_tc_turbo() &&
           ((mzf_stage == MZF_STAGE_DATA) ||
            (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA));
}

static uint16_t mzf_trailing_pulses(void)
{
    return mzf_stage_uses_tc_trailing() ?
        MZF_TC_LOADER_TRAILING_SHORT_PULSES :
        MZF_MZ800_TRAILING_LONG_PULSES;
}
static void mzf_begin_normal_stage(mzf_stage_t stage)
{
    mzf_stage = stage;
    mzf_normal_step = MZF_STEP_BEGIN;
    mzf_normal_loop = 0U;
    mzf_normal_bytes_remaining = 0UL;
    mzf_normal_checksum = 0U;
    mzf_normal_data = 0U;
    mzf_normal_bits_remaining = 0U;
    mzf_normal_checksum_byte_index = 0U;
    mzf_normal_header_preamble = (stage == MZF_STAGE_HEADER);
    mzf_boundary_waiting = false;
    mzf_motor_low_seen = 0U;
    mzf_boundary_auto_timer_armed = false;
    mzf_boundary_auto_start_ms = 0U;
    mzf_paused_mid_pulse = false;
    mzf_timer_phase_high = false;
    mzf_timer_phase_low_first = false;
    mzf_current_pulse_is_long = false;

    if (stage == MZF_STAGE_HEADER)
    {
        mzf_header_offset = 0U;
    }
}

static uint32_t mzf_stage_byte_count(void)
{
    if (mzf_stage == MZF_STAGE_HEADER) return MZF_HEADER_BYTES;
    if ((mzf_stage == MZF_STAGE_DATA) ||
        (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)) return mzf_record_data_length;
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

    if ((mzf_stage == MZF_STAGE_DATA) ||
        (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA))
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
                mzf_normal_loop = mzf_gap_short_pulses();
                mzf_normal_step = MZF_STEP_GAP;
                continue;

            case MZF_STEP_GAP:
                if (mzf_normal_loop == 0U)
                {
                    mzf_normal_loop = mzf_mark_long_pulses();
                    mzf_normal_step = MZF_STEP_TAPE_MARK_LONG;
                    continue;
                }
                mzf_set_short_pulse(high_ticks, low_ticks);
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_LONG:
                if (mzf_normal_loop == 0U)
                {
                    mzf_normal_loop = mzf_mark_short_pulses();
                    mzf_normal_step = MZF_STEP_TAPE_MARK_SHORT;
                    continue;
                }
                mzf_set_long_pulse(high_ticks, low_ticks);
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_SHORT:
                if (mzf_normal_loop == 0U)
                {
                    mzf_normal_loop = mzf_mark_final_long_pulses();
                    mzf_normal_step = MZF_STEP_TAPE_MARK_FINAL;
                    continue;
                }
                mzf_set_short_pulse(high_ticks, low_ticks);
                mzf_normal_loop--;
                return true;

            case MZF_STEP_TAPE_MARK_FINAL:
                if (mzf_normal_loop == 0U)
                {
                    byte_count = mzf_stage_byte_count();
                    mzf_normal_bytes_remaining = byte_count;
                    mzf_normal_checksum = 0U;
                    mzf_normal_checksum_byte_index = 0U;
                    mzf_normal_step = MZF_STEP_BYTE_LOAD;
                    continue;
                }
                mzf_set_long_pulse(high_ticks, low_ticks);
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
                    mzf_set_long_pulse(high_ticks, low_ticks);
                    mzf_normal_checksum++;
                }
                else
                {
                    mzf_set_short_pulse(high_ticks, low_ticks);
                }
                mzf_normal_data <<= 1U;
                mzf_normal_bits_remaining--;
                return true;

            case MZF_STEP_BYTE_STOP:
                /* MZ-800 ROM format: one long stop pulse follows every byte. */
                mzf_set_long_pulse(high_ticks, low_ticks);
                mzf_normal_step = MZF_STEP_BYTE_LOAD;
                return true;

            case MZF_STEP_CHECKSUM_LOAD:
                if (mzf_normal_checksum_byte_index >= 2U)
                {
                    mzf_normal_loop = mzf_trailing_pulses();
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
                    mzf_set_long_pulse(high_ticks, low_ticks);
                }
                else
                {
                    mzf_set_short_pulse(high_ticks, low_ticks);
                }
                mzf_normal_data <<= 1U;
                mzf_normal_bits_remaining--;
                return true;

            case MZF_STEP_CHECKSUM_STOP:
                mzf_set_long_pulse(high_ticks, low_ticks);
                mzf_normal_step = MZF_STEP_CHECKSUM_LOAD;
                return true;

            case MZF_STEP_TRAILING_LONGS:
                if (mzf_normal_loop == 0U)
                {
                    mzf_normal_step = MZF_STEP_BOUNDARY;
                    continue;
                }
                if (mzf_stage_uses_tc_trailing())
                {
                    mzf_set_short_pulse(high_ticks, low_ticks);
                }
                else
                {
                    mzf_set_long_pulse(high_ticks, low_ticks);
                }
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
                if ((mzf_stage == MZF_STAGE_DATA) && mzf_loader_is_ul_active())
                {
                    mzf_boundary_waiting = true;
                    mzf_stop_timer_from_isr();
                    mz_read_set_fast_from_isr(0U);
                    return false;
                }

                if ((mzf_stage == MZF_STAGE_TAPE_TURBO_DATA) ||
                    ((mzf_stage == MZF_STAGE_DATA) &&
                     (mzf_record_data_file_end >= mzf_file_size)))
                {
                    mzf_stop_timer_from_isr();
                    mz_read_set_fast_from_isr(0U);
                    mzf_state = MZF_PLAYBACK_FINISHED;
                    return false;
                }

                mzf_boundary_waiting = true;
                mzf_stop_timer_from_isr();
                mzf_read_wave_set_from_isr(0U);
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

    mzf_current_pulse_is_long = (high_ticks == mzf_long_high_ticks());
    if (mzf_stage_uses_low_first_timing())
    {
        mzf_timer_phase_high = false;
        mzf_timer_phase_low_first = true;
        mzf_read_wave_set_from_isr(0U);
        mzf_paused_remaining_ticks = low_ticks;
        mzf_timer_start_phase_from_isr(low_ticks);
    }
    else
    {
        mzf_timer_phase_high = true;
        mzf_timer_phase_low_first = false;
        mzf_read_wave_set_from_isr(1U);
        mzf_paused_remaining_ticks = high_ticks;
        mzf_timer_start_phase_from_isr(high_ticks);
    }
    return true;
}

/* Low ticks are determined from the current high/short-or-long pulse class. */
static uint16_t mzf_current_low_ticks(void)
{
    return mzf_current_pulse_is_long ?
        mzf_long_low_ticks() : mzf_short_low_ticks();
}

static uint16_t mzf_current_high_ticks(void)
{
    return mzf_current_pulse_is_long ?
        mzf_long_high_ticks() : mzf_short_high_ticks();
}

static bool mzf_start_normal_output(void)
{
    /* The hardware-proven MZ-700 WAV keeps READ low for about 400 ms
       between the synthetic header and the first FAST3 leader pulse.
       A real MOTOR low/high cycle can be much shorter, so apply this once
       when the accelerated payload stage is first started. */
    if (mzf_loader_is_mz700_fast3() &&
        (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA) &&
        (mzf_normal_step == MZF_STEP_BEGIN))
    {
        mz_read_set(false);
        delay(MZF_MZ700_FAST3_START_DELAY_MS);
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        mzf_start_next_normal_pulse_from_isr();
    }
    return (mzf_state == MZF_PLAYBACK_RUNNING) && !mzf_boundary_waiting;
}

static bool mzf_start_ultrafast_output(void)
{
    if (!mzf_loader_begin())
    {
        mzf_set_error(mzf_loader_get_error_text(), MZF_PLAYBACK_IO_ERROR);
        return false;
    }
    return true;
}

static bool mzf_prepare_tape_turbo_payload(void)
{
    if (!sdcard_file_seek(mzf_original_data_offset))
    {
        mzf_set_error_P(PSTR("TURB SEEK"), MZF_PLAYBACK_IO_ERROR);
        return false;
    }

    mzf_record_data_length = mzf_original_data_length;
    mzf_record_data_file_end = mzf_original_data_offset + mzf_original_data_length;
    mzf_record_data_read = 0UL;
    mzf_fifo_reset();
    return mzf_prefill_data();
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
        if (mzf_loader_is_mz700_fast3())
        {
            if (!mzf_prepare_tape_turbo_payload())
            {
                return false;
            }
            mzf_begin_normal_stage(MZF_STAGE_TAPE_TURBO_DATA);
            return true;
        }
        if (mzf_loader_is_header_only())
        {
            mzf_stage = MZF_STAGE_ULTRAFAST;
            mzf_boundary_waiting = false;
            mzf_motor_low_seen = 0U;
            mzf_boundary_auto_timer_armed = false;
            mzf_boundary_auto_start_ms = 0U;
            return true;
        }
        if (mzf_loader_is_ic_turbo())
        {
            if (!mzf_prepare_tape_turbo_payload())
            {
                return false;
            }
            mzf_begin_normal_stage(MZF_STAGE_TAPE_TURBO_DATA);
            return true;
        }
        mzf_begin_normal_stage(MZF_STAGE_DATA);
        return true;
    }

    if ((mzf_stage == MZF_STAGE_DATA) && mzf_loader_is_tape_turbo())
    {
        if (!mzf_prepare_tape_turbo_payload())
        {
            return false;
        }
        mzf_begin_normal_stage(MZF_STAGE_TAPE_TURBO_DATA);
        return true;
    }

    if ((mzf_stage != MZF_STAGE_DATA) &&
        (mzf_stage != MZF_STAGE_TAPE_TURBO_DATA))
    {
        mzf_set_error_P(PSTR("MZF STATE"), MZF_PLAYBACK_BAD_FILE);
        return false;
    }

    if ((mzf_stage == MZF_STAGE_DATA) && mzf_loader_is_ul_active())
    {
        mzf_stage = MZF_STAGE_ULTRAFAST;
        mzf_boundary_waiting = false;
        mzf_motor_low_seen = 0U;
        mzf_boundary_auto_timer_armed = false;
        mzf_boundary_auto_start_ms = 0U;
        return true;
    }

    if (mzf_format == FILE_FORMAT_MZT)
    {
        if (sdcard_file_position() < mzf_file_size)
        {
            return mzf_start_next_mzt_record();
        }
    }
    else if ((mzf_stage == MZF_STAGE_DATA) &&
             (sdcard_file_position() < mzf_file_size))
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
    bool ul_loader_boundary;
    bool tc_turbo_boundary;
    uint16_t now;

    if (!mzf_boundary_waiting || (mzf_state != MZF_PLAYBACK_RUNNING))
    {
        return;
    }

    ul_loader_boundary = mzf_loader_is_ul_active() &&
                         (((mzf_stage == MZF_STAGE_DATA) &&
                           !mzf_loader_is_header_only()) ||
                          ((mzf_stage == MZF_STAGE_HEADER) &&
                           mzf_loader_is_header_only()));
    tc_turbo_boundary = (mzf_stage == MZF_STAGE_DATA) &&
                        mzf_loader_is_tc_turbo();

    /* Let the controller preserve the native MOTOR-low pause behavior. */
    if (!ul_loader_boundary && !mz_motor_get())
    {
        return;
    }

    if (!ul_loader_boundary)
    {
        now = (uint16_t)millis();
        if (!mzf_boundary_auto_timer_armed)
        {
            mzf_boundary_auto_start_ms = now;
            mzf_boundary_auto_timer_armed = true;

            /* A low edge observed at timer precision has already completed the
               legacy boundary; do not add an unnecessary silent delay. */
            if ((mzf_motor_low_seen == 0U) || tc_turbo_boundary)
            {
                return;
            }
        }
        else if (((mzf_motor_low_seen == 0U) || tc_turbo_boundary) &&
                 ((uint16_t)(now - mzf_boundary_auto_start_ms) <
                  mzf_boundary_auto_continue_ms()))
        {
            return;
        }
    }

    if (!mzf_advance_after_boundary())
    {
        return;
    }

    if (mzf_state == MZF_PLAYBACK_FINISHED)
    {
        return;
    }

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        if (!mzf_start_ultrafast_output() && (mzf_state == MZF_PLAYBACK_RUNNING))
        {
            mzf_set_error_P(PSTR("UL START"), MZF_PLAYBACK_BAD_FILE);
        }
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
    mzf_wave_invert_signal = false;
    mzf_state = MZF_PLAYBACK_STOPPED;
    mzf_error_text[0] = '\0';
    mzf_format = FILE_FORMAT_UNKNOWN;
    mzf_file_size = 0UL;
    mzf_total_duration_ms = 0UL;
    mzf_record_data_length = 0UL;
    mzf_record_data_file_end = 0UL;
    mzf_record_data_read = 0UL;
    mzf_original_data_offset = 0UL;
    mzf_original_data_length = 0UL;
    mzf_stage = MZF_STAGE_NONE;
    mzf_boundary_waiting = false;
    mzf_motor_low_seen = 0U;
    mzf_boundary_auto_timer_armed = false;
    mzf_boundary_auto_start_ms = 0U;
    mzf_timer_phase_high = false;
    mzf_timer_phase_low_first = false;
    mzf_current_pulse_is_long = false;
    mzf_paused_mid_pulse = false;
    mzf_paused_remaining_ticks = 0U;
    mzf_fifo_reset();
    mzf_loader_reset();
}

bool mzf_playback_prepare(const char *path,
                          file_format_t format,
                          loader_mode_t loader_mode)
{
    mzf_playback_stop();
    mzf_error_text[0] = '\0';
    mzf_wave_invert_signal = false;

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

    mzf_original_data_offset = sdcard_file_position();
    mzf_original_data_length = mzf_record_data_length;

    if (mzf_loader_prepare(format, loader_mode, mzf_header,
                              mzf_file_size, sdcard_file_position()))
    {
        mzf_wave_invert_signal = mzf_loader_is_tc_turbo();
        if (!mzf_loader_patch_loader_header(mzf_header))
        {
            sdcard_file_close();
            mzf_set_error_P(PSTR("LDR HEADER"), MZF_PLAYBACK_BAD_FILE);
            return false;
        }
        if (!mzf_loader_is_header_only() &&
            !mzf_loader_is_ic_turbo() &&
            !mzf_loader_is_mz700_fast3() &&
            !mzf_prepare_loader_block_data())
        {
            sdcard_file_close();
            return false;
        }
        mzf_total_duration_ms = 0UL;
    }
    else if (!mzf_prefill_data())
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
    if (mzf_stage != MZF_STAGE_ULTRAFAST)
    {
        mz_sense_set(false);
    }

    if (mzf_paused_mid_pulse)
    {
        /* This low MOTOR belonged to a mid-block pause, not to the next
           header/data boundary. */
        mzf_motor_low_seen = 0U;
        mzf_paused_mid_pulse = false;
        mzf_timer_start_resume(mzf_paused_remaining_ticks);
        return true;
    }

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        return mzf_start_ultrafast_output();
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

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        mzf_state = MZF_PLAYBACK_PAUSED;
        return true;
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
    mzf_boundary_auto_start_ms = 0U;
    mzf_timer_phase_high = false;
    mzf_timer_phase_low_first = false;
    mzf_current_pulse_is_long = false;
    mzf_paused_mid_pulse = false;
    mzf_paused_remaining_ticks = 0U;
    mzf_file_size = 0UL;
    mzf_total_duration_ms = 0UL;
    mzf_wave_invert_signal = false;
    mzf_record_data_length = 0UL;
    mzf_record_data_file_end = 0UL;
    mzf_record_data_read = 0UL;
    mzf_original_data_offset = 0UL;
    mzf_original_data_length = 0UL;
    mzf_loader_reset();
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

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        if (!mzf_loader_pump(MZF_LOADER_PUMP_BYTES))
        {
            mzf_set_error(mzf_loader_get_error_text(), MZF_PLAYBACK_IO_ERROR);
            return;
        }
        if (mzf_loader_is_finished())
        {
            mzf_state = MZF_PLAYBACK_FINISHED;
            mz_sense_set(true);
        }
        return;
    }

    if (((mzf_stage == MZF_STAGE_DATA) ||
         (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)) && !mzf_boundary_waiting &&
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

    if ((mzf_stage != MZF_STAGE_DATA) &&
        (mzf_stage != MZF_STAGE_TAPE_TURBO_DATA))
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

static uint16_t mzf_progress_gap_pulses(mzf_stage_t stage)
{
    if (stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_GAP_SHORT_PULSES;
    }
    if (stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        switch (mzf_loader_get_variant())
        {
            case MZF_LOADER_VARIANT_TC_1_2:
                return MZF_TC_1_2_TURBO_GAP_SHORT_PULSES;
            case MZF_LOADER_VARIANT_TC_1_3:
                return MZF_TC_1_3_TURBO_GAP_SHORT_PULSES;
            default:
                return MZF_IC_TURBO_GAP_SHORT_PULSES;
        }
    }
    return MZF_MZ800_SHORT_GAP_SHORT_PULSES;
}

static uint16_t mzf_progress_mark_long_pulses(mzf_stage_t stage)
{
    if (stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_MARK_LONG_PULSES;
    }
    return (stage == MZF_STAGE_TAPE_TURBO_DATA) ?
        MZF_IC_TURBO_MARK_LONG_PULSES :
        MZF_MZ800_SHORT_MARK_LONG_PULSES;
}

static uint16_t mzf_progress_mark_short_pulses(mzf_stage_t stage)
{
    if (stage == MZF_STAGE_HEADER)
    {
        return MZF_MZ800_LONG_MARK_SHORT_PULSES;
    }
    return (stage == MZF_STAGE_TAPE_TURBO_DATA) ?
        MZF_IC_TURBO_MARK_SHORT_PULSES :
        MZF_MZ800_SHORT_MARK_SHORT_PULSES;
}

static uint16_t mzf_progress_mark_final_pulses(mzf_stage_t stage)
{
    return (stage == MZF_STAGE_TAPE_TURBO_DATA) ?
        MZF_IC_TURBO_MARK_FINAL_LONG_PULSES :
        MZF_MZ800_TAPE_MARK_FINAL_LONG_PULSES;
}

static uint16_t mzf_progress_trailing_pulses(mzf_stage_t stage)
{
    return (mzf_loader_is_tc_turbo() &&
            ((stage == MZF_STAGE_DATA) ||
             (stage == MZF_STAGE_TAPE_TURBO_DATA))) ?
        MZF_TC_LOADER_TRAILING_SHORT_PULSES :
        MZF_MZ800_TRAILING_LONG_PULSES;
}

static uint16_t mzf_progress_done_pulses(uint16_t total, uint16_t remaining)
{
    return (remaining >= total) ? 0U : (uint16_t)(total - remaining);
}

static uint32_t mzf_progress_stage_total_units(mzf_stage_t stage,
                                               uint32_t byte_count)
{
    return (uint32_t)mzf_progress_gap_pulses(stage) +
           (uint32_t)mzf_progress_mark_long_pulses(stage) +
           (uint32_t)mzf_progress_mark_short_pulses(stage) +
           (uint32_t)mzf_progress_mark_final_pulses(stage) +
           (byte_count * 9UL) + 18UL +
           (uint32_t)mzf_progress_trailing_pulses(stage);
}

static uint32_t mzf_progress_byte_units(mzf_normal_step_t step,
                                        uint32_t bytes_read,
                                        uint32_t byte_count,
                                        uint8_t bits_remaining)
{
    if (bytes_read > byte_count) bytes_read = byte_count;

    if (((step == MZF_STEP_BYTE_BITS) ||
         (step == MZF_STEP_BYTE_STOP)) && (bytes_read != 0UL))
    {
        uint32_t done = (bytes_read - 1UL) * 9UL;
        done += (step == MZF_STEP_BYTE_BITS) ?
            (uint32_t)(8U - bits_remaining) : 8UL;
        return done;
    }

    return bytes_read * 9UL;
}

static uint32_t mzf_progress_checksum_units(mzf_normal_step_t step,
                                            uint8_t checksum_byte_index,
                                            uint8_t bits_remaining)
{
    uint32_t done;

    if (checksum_byte_index > 2U) checksum_byte_index = 2U;
    done = (uint32_t)checksum_byte_index * 9UL;

    if (((step == MZF_STEP_CHECKSUM_BITS) ||
         (step == MZF_STEP_CHECKSUM_STOP)) && (checksum_byte_index != 0U))
    {
        done = (uint32_t)(checksum_byte_index - 1U) * 9UL;
        done += (step == MZF_STEP_CHECKSUM_BITS) ?
            (uint32_t)(8U - bits_remaining) : 8UL;
    }

    return (done > 18UL) ? 18UL : done;
}

static uint32_t mzf_progress_stage_done_units(mzf_stage_t stage,
                                              mzf_normal_step_t step,
                                              uint16_t loop,
                                              uint32_t bytes_read,
                                              uint32_t byte_count,
                                              uint8_t bits_remaining,
                                              uint8_t checksum_byte_index)
{
    uint16_t gap = mzf_progress_gap_pulses(stage);
    uint16_t mark_long = mzf_progress_mark_long_pulses(stage);
    uint16_t mark_short = mzf_progress_mark_short_pulses(stage);
    uint16_t mark_final = mzf_progress_mark_final_pulses(stage);
    uint32_t preamble = (uint32_t)gap + (uint32_t)mark_long +
                        (uint32_t)mark_short + (uint32_t)mark_final;
    uint32_t data_units = byte_count * 9UL;

    switch (step)
    {
        case MZF_STEP_BEGIN:
            return 0UL;
        case MZF_STEP_GAP:
            return (uint32_t)mzf_progress_done_pulses(gap, loop);
        case MZF_STEP_TAPE_MARK_LONG:
            return (uint32_t)gap +
                   (uint32_t)mzf_progress_done_pulses(mark_long, loop);
        case MZF_STEP_TAPE_MARK_SHORT:
            return (uint32_t)gap + (uint32_t)mark_long +
                   (uint32_t)mzf_progress_done_pulses(mark_short, loop);
        case MZF_STEP_TAPE_MARK_FINAL:
            return (uint32_t)gap + (uint32_t)mark_long +
                   (uint32_t)mark_short +
                   (uint32_t)mzf_progress_done_pulses(mark_final, loop);
        case MZF_STEP_BYTE_LOAD:
        case MZF_STEP_BYTE_BITS:
        case MZF_STEP_BYTE_STOP:
            return preamble + mzf_progress_byte_units(step, bytes_read,
                                                       byte_count,
                                                       bits_remaining);
        case MZF_STEP_CHECKSUM_LOAD:
        case MZF_STEP_CHECKSUM_BITS:
        case MZF_STEP_CHECKSUM_STOP:
            return preamble + data_units +
                   mzf_progress_checksum_units(step, checksum_byte_index,
                                               bits_remaining);
        case MZF_STEP_TRAILING_LONGS:
            return preamble + data_units + 18UL +
                   (uint32_t)mzf_progress_done_pulses(
                       mzf_progress_trailing_pulses(stage), loop);
        case MZF_STEP_BOUNDARY:
            return mzf_progress_stage_total_units(stage, byte_count);
        default:
            return 0UL;
    }
}

uint8_t mzf_playback_get_progress_percent(void)
{
    uint32_t percent;
    uint32_t total;
    uint32_t done = 0UL;
    uint32_t header_units;
    uint32_t loader_units = 0UL;
    uint32_t data_units = 0UL;
    uint32_t stage_bytes_read;
    uint16_t read_sequence;
    uint16_t loop;
    uint8_t header_offset;
    uint8_t bits_remaining;
    uint8_t checksum_byte_index;
    mzf_stage_t stage;
    mzf_normal_step_t step;

    if (!mzf_loader_is_active())
    {
        return 0U;
    }

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        return mzf_loader_get_progress_percent();
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        stage = mzf_stage;
        step = mzf_normal_step;
        loop = mzf_normal_loop;
        header_offset = mzf_header_offset;
        read_sequence = mzf_fifo_read_sequence;
        bits_remaining = mzf_normal_bits_remaining;
        checksum_byte_index = mzf_normal_checksum_byte_index;
    }

    header_units = mzf_progress_stage_total_units(MZF_STAGE_HEADER,
                                                  MZF_HEADER_BYTES);
    total = header_units;

    if (mzf_loader_is_ic_turbo() || mzf_loader_is_mz700_fast3())
    {
        data_units = mzf_progress_stage_total_units(MZF_STAGE_TAPE_TURBO_DATA,
                                                    mzf_original_data_length);
        total += data_units;
    }
    else if (!mzf_loader_is_header_only())
    {
        loader_units = mzf_progress_stage_total_units(
            MZF_STAGE_DATA, (uint32_t)mzf_loader_get_loader_size());
        total += loader_units;
        if (mzf_loader_is_tc_turbo())
        {
            data_units = mzf_progress_stage_total_units(
                MZF_STAGE_TAPE_TURBO_DATA, mzf_original_data_length);
            total += data_units;
        }
    }

    if (total == 0UL)
    {
        return 0U;
    }

    if (stage == MZF_STAGE_HEADER)
    {
        done = mzf_progress_stage_done_units(stage, step, loop,
                                             (uint32_t)header_offset,
                                             MZF_HEADER_BYTES,
                                             bits_remaining,
                                             checksum_byte_index);
    }
    else if (stage == MZF_STAGE_DATA)
    {
        stage_bytes_read = (uint32_t)read_sequence;
        done = header_units +
               mzf_progress_stage_done_units(stage, step, loop,
                                             stage_bytes_read,
                                             (uint32_t)mzf_loader_get_loader_size(),
                                             bits_remaining,
                                             checksum_byte_index);
    }
    else if (stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        stage_bytes_read = (uint32_t)read_sequence;
        done = header_units + loader_units +
               mzf_progress_stage_done_units(stage, step, loop,
                                             stage_bytes_read,
                                             mzf_original_data_length,
                                             bits_remaining,
                                             checksum_byte_index);
    }
    else if (stage != MZF_STAGE_NONE)
    {
        done = total;
    }

    if (done > total) done = total;
    percent = (done * 100UL) / total;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}
mzf_playback_phase_t mzf_playback_get_progress_phase(void)
{
    if (!mzf_loader_is_active())
    {
        return MZF_PLAYBACK_PHASE_NORMAL;
    }

    if (mzf_stage == MZF_STAGE_ULTRAFAST)
    {
        return MZF_PLAYBACK_PHASE_ULTRAFAST_DATA;
    }

    if (mzf_stage == MZF_STAGE_TAPE_TURBO_DATA)
    {
        if (mzf_loader_is_mz700_fast3())
        {
            return mzf_loader_is_mz700_fast3_high() ?
                MZF_PLAYBACK_PHASE_MZ700_FAST3_HIGH :
                MZF_PLAYBACK_PHASE_MZ700_FAST3_LOW;
        }
        return mzf_loader_is_tc_turbo() ?
            MZF_PLAYBACK_PHASE_TC_TURBO_DATA :
            MZF_PLAYBACK_PHASE_IC_TURBO_DATA;
    }

    if (mzf_loader_is_ic_turbo())
    {
        return MZF_PLAYBACK_PHASE_IC_TURBO_DATA;
    }

    if (mzf_loader_is_tc_turbo())
    {
        return MZF_PLAYBACK_PHASE_TC_TURBO_LOADER;
    }

    switch (mzf_loader_get_variant())
    {
        case MZF_LOADER_VARIANT_LOW:
            return MZF_PLAYBACK_PHASE_ULTRAFAST_LOADER_LOW;
        case MZF_LOADER_VARIANT_HIGH:
            return MZF_PLAYBACK_PHASE_ULTRAFAST_LOADER_HIGH;
        case MZF_LOADER_VARIANT_MZ800_HEADER:
            return MZF_PLAYBACK_PHASE_ULTRAFAST_HEADER;
        case MZF_LOADER_VARIANT_MZ700_FAST3_LOW:
            return MZF_PLAYBACK_PHASE_MZ700_FAST3_LOW;
        case MZF_LOADER_VARIANT_MZ700_FAST3_HIGH:
            return MZF_PLAYBACK_PHASE_MZ700_FAST3_HIGH;
        default:
            return MZF_PLAYBACK_PHASE_NORMAL;
    }
}

bool mzf_playback_is_ul_loader_active(void)
{
    return mzf_loader_is_ul_active();
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
        low_ticks = mzf_current_low_ticks();
        mzf_read_wave_set_from_isr(0U);
        mzf_timer_start_phase_from_isr(low_ticks);
        return true;
    }

    if (mzf_timer_phase_low_first)
    {
        mzf_timer_phase_low_first = false;
        mzf_read_wave_set_from_isr(1U);
        mzf_timer_start_phase_from_isr(mzf_current_high_ticks());
        return true;
    }

    (void)mzf_start_next_normal_pulse_from_isr();
    return true;
}
