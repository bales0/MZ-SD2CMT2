#include "mz_tape_decoder.h"

#include <stddef.h>

#define DECODER_COUNT 2U
#define MIN_LEADER_PULSES 16U
#define MIN_MARK_PULSES 12U
#define MAX_MARK_PULSES 48U
#define FINAL_MARK_PULSES 2U
#define MAX_HALF_UNITS 1024U

typedef enum
{
    DECODE_SEARCH_LEADER = 0,
    DECODE_MARK_LONG,
    DECODE_MARK_SHORT,
    DECODE_MARK_FINAL,
    DECODE_DATA,
    DECODE_DUPLICATE_GAP
} decode_state_t;

typedef enum
{
    DECODER_MODE_STOPPED = 0,
    DECODER_MODE_HEADER,
    DECODER_MODE_DATA
} decoder_mode_t;

typedef struct
{
    decode_state_t state;
    uint16_t short_x8;
    uint16_t leader_pulses;
    uint8_t mark_pulses;
    uint8_t final_pulses;
    uint8_t bit_count;
    uint8_t byte_value;
    uint32_t byte_index;
    uint32_t expected_bytes;
    uint16_t checksum;
    uint16_t recorded_checksum;
    uint8_t copy_index;
} pulse_decoder_t;

static decoder_mode_t decoder_mode = DECODER_MODE_STOPPED;
static pulse_decoder_t decoders[DECODER_COUNT];
static uint8_t header_buffers[DECODER_COUNT][MZ_TAPE_HEADER_BYTES];
static const uint8_t *validated_header = NULL;
static uint8_t header_start_level = 0U;
static uint8_t selected_start_level = 0U;
static bool selected_level_valid = false;
static bool have_previous_half = false;
static uint16_t previous_half_units = 0U;
static uint8_t previous_half_level = 0U;
static mz_tape_decoder_event_t pending_event;

static uint8_t popcount8(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static void reset_candidate(pulse_decoder_t *decoder, uint16_t seed_units)
{
    if (decoder == NULL) return;
    decoder->state = DECODE_SEARCH_LEADER;
    decoder->short_x8 = (seed_units <= (MAX_HALF_UNITS * 2U)) ?
        (uint16_t)(seed_units * 8U) : 0U;
    decoder->leader_pulses = (decoder->short_x8 != 0U) ? 1U : 0U;
    decoder->mark_pulses = 0U;
    decoder->final_pulses = 0U;
    decoder->bit_count = 0U;
    decoder->byte_value = 0U;
    decoder->byte_index = 0UL;
    decoder->checksum = 0U;
    decoder->recorded_checksum = 0U;
    decoder->copy_index = 0U;
}

static bool is_leader_pulse(const pulse_decoder_t *decoder,
                            uint16_t duration_units)
{
    uint32_t scaled = (uint32_t)duration_units * 8UL;
    uint32_t average;
    uint32_t difference;

    if ((decoder == NULL) || (decoder->short_x8 == 0U)) return false;
    average = decoder->short_x8;
    difference = (scaled >= average) ? (scaled - average) : (average - scaled);
    return (((scaled * 4UL) >= (average * 3UL)) &&
            ((scaled * 4UL) <= (average * 5UL))) ||
           (difference <= 8UL);
}

static int8_t classify_pulse(const pulse_decoder_t *decoder,
                             uint16_t duration_units)
{
    uint32_t scaled = (uint32_t)duration_units * 8UL;
    uint32_t average;

    if ((decoder == NULL) || (decoder->short_x8 == 0U)) return -1;
    average = decoder->short_x8;
    if (((scaled * 2UL) < average) || (scaled > (average * 3UL))) return -1;
    return ((scaled * 20UL) < (average * 29UL)) ? 0 : 1;
}

static void publish_event(mz_tape_decoder_event_type_t type,
                          uint8_t value,
                          uint32_t byte_index,
                          const pulse_decoder_t *decoder)
{
    /* A caller consumes after every foreground interval. Preserve the first
       event if malformed input somehow tries to publish two at once. */
    if (pending_event.type != MZ_TAPE_DECODER_EVENT_NONE) return;
    pending_event.type = type;
    pending_event.value = value;
    pending_event.byte_index = byte_index;
    pending_event.calculated_checksum = decoder->checksum;
    pending_event.recorded_checksum = decoder->recorded_checksum;
    pending_event.leader_pulses = decoder->leader_pulses;
    pending_event.copy_index = decoder->copy_index;
}

static void begin_duplicate_gap(pulse_decoder_t *decoder,
                                uint32_t expected_bytes)
{
    decoder->state = DECODE_DUPLICATE_GAP;
    decoder->leader_pulses = 0U;
    decoder->mark_pulses = 0U;
    decoder->final_pulses = 0U;
    decoder->bit_count = 0U;
    decoder->byte_value = 0U;
    decoder->byte_index = 0UL;
    decoder->expected_bytes = expected_bytes;
    decoder->checksum = 0U;
    decoder->recorded_checksum = 0U;
    decoder->copy_index = 1U;
}

static void accept_byte(pulse_decoder_t *decoder,
                        uint8_t decoder_index,
                        uint8_t value)
{
    uint32_t index = decoder->byte_index;

    if (index < decoder->expected_bytes)
    {
        decoder->checksum = (uint16_t)(decoder->checksum + popcount8(value));
        if (decoder_mode == DECODER_MODE_HEADER)
        {
            header_buffers[decoder_index][(uint8_t)index] = value;
        }
        else
        {
            publish_event(MZ_TAPE_DECODER_EVENT_DATA_BYTE, value, index,
                          decoder);
        }
    }
    else
    {
        decoder->recorded_checksum =
            (uint16_t)((decoder->recorded_checksum << 8U) | value);
    }

    decoder->byte_index++;
    if (decoder->byte_index == (decoder->expected_bytes + 2UL))
    {
        bool valid = decoder->recorded_checksum == decoder->checksum;
        if (decoder_mode == DECODER_MODE_HEADER)
        {
            if (valid)
            {
                validated_header = header_buffers[decoder_index];
                header_start_level = decoder_index;
                selected_start_level = decoder_index;
                selected_level_valid = true;
                decoder_mode = DECODER_MODE_STOPPED;
                publish_event(MZ_TAPE_DECODER_EVENT_HEADER_VALID, 0U, 0UL,
                              decoder);
                return;
            }
            /* Native MZ700 repeats the block without a new leader/mark.  Keep
               this candidate's calibrated short period and wait for the
               documented 256-short separator. */
            begin_duplicate_gap(decoder, MZ_TAPE_HEADER_BYTES);
            return;
        }

        publish_event(valid ? MZ_TAPE_DECODER_EVENT_BLOCK_VALID :
                              MZ_TAPE_DECODER_EVENT_BLOCK_INVALID,
                      0U, decoder->expected_bytes, decoder);
        decoder_mode = DECODER_MODE_STOPPED;
    }
}

static void accept_data_pulse(pulse_decoder_t *decoder,
                              uint8_t decoder_index,
                              uint8_t pulse_class)
{
    if (decoder->bit_count < 8U)
    {
        decoder->byte_value =
            (uint8_t)((decoder->byte_value << 1U) | pulse_class);
        decoder->bit_count++;
        return;
    }
    if (pulse_class != 1U)
    {
        reset_candidate(decoder, 0U);
        return;
    }
    accept_byte(decoder, decoder_index, decoder->byte_value);
    decoder->byte_value = 0U;
    decoder->bit_count = 0U;
}

static void feed_pulse(pulse_decoder_t *decoder,
                       uint8_t decoder_index,
                       uint16_t duration_units)
{
    int8_t pulse_class;

    if ((decoder == NULL) || (decoder_mode == DECODER_MODE_STOPPED)) return;
    if (decoder->state == DECODE_DUPLICATE_GAP)
    {
        pulse_class = classify_pulse(decoder, duration_units);
        if (pulse_class == 0)
        {
            if (decoder->leader_pulses < 256U) decoder->leader_pulses++;
            if (decoder->leader_pulses == 256U)
            {
                decoder->state = DECODE_DATA;
                decoder->leader_pulses = 0U;
            }
        }
        else if (decoder->leader_pulses < 128U)
        {
            /* Ignore checksum trailing longs before the separator. */
            decoder->leader_pulses = 0U;
        }
        else
        {
            reset_candidate(decoder, duration_units);
        }
        return;
    }
    if (decoder->state == DECODE_SEARCH_LEADER)
    {
        uint32_t scaled;
        if (decoder->short_x8 == 0U)
        {
            reset_candidate(decoder, duration_units);
            return;
        }
        scaled = (uint32_t)duration_units * 8UL;
        if (is_leader_pulse(decoder, duration_units))
        {
            decoder->short_x8 = (uint16_t)
                ((((uint32_t)decoder->short_x8 * 7UL) + scaled + 4UL) / 8UL);
            if (decoder->leader_pulses != 0xFFFFU) decoder->leader_pulses++;
            return;
        }
        pulse_class = classify_pulse(decoder, duration_units);
        if ((decoder->leader_pulses >= MIN_LEADER_PULSES) &&
            (pulse_class == 1))
        {
            decoder->state = DECODE_MARK_LONG;
            decoder->mark_pulses = 1U;
            return;
        }
        reset_candidate(decoder, duration_units);
        return;
    }

    pulse_class = classify_pulse(decoder, duration_units);
    if (pulse_class < 0)
    {
        reset_candidate(decoder, duration_units);
        return;
    }
    if (decoder->state == DECODE_MARK_LONG)
    {
        if (pulse_class == 1)
        {
            if (decoder->mark_pulses != 0xFFU) decoder->mark_pulses++;
            return;
        }
        if ((decoder->mark_pulses >= MIN_MARK_PULSES) &&
            (decoder->mark_pulses <= MAX_MARK_PULSES))
        {
            decoder->state = DECODE_MARK_SHORT;
            decoder->mark_pulses = 1U;
            return;
        }
        reset_candidate(decoder, duration_units);
        return;
    }
    if (decoder->state == DECODE_MARK_SHORT)
    {
        if (pulse_class == 0)
        {
            if (decoder->mark_pulses != 0xFFU) decoder->mark_pulses++;
            return;
        }
        if ((decoder->mark_pulses >= MIN_MARK_PULSES) &&
            (decoder->mark_pulses <= MAX_MARK_PULSES))
        {
            decoder->state = DECODE_MARK_FINAL;
            decoder->final_pulses = 1U;
            return;
        }
        reset_candidate(decoder, duration_units);
        return;
    }
    if (decoder->state == DECODE_MARK_FINAL)
    {
        if (pulse_class != 1)
        {
            reset_candidate(decoder, duration_units);
            return;
        }
        decoder->final_pulses++;
        if (decoder->final_pulses == FINAL_MARK_PULSES)
        {
            decoder->state = DECODE_DATA;
        }
        return;
    }
    accept_data_pulse(decoder, decoder_index, (uint8_t)pulse_class);
}

void mz_tape_decoder_begin_header(void)
{
    decoder_mode = DECODER_MODE_HEADER;
    validated_header = NULL;
    selected_level_valid = false;
    have_previous_half = false;
    previous_half_units = 0U;
    pending_event.type = MZ_TAPE_DECODER_EVENT_NONE;
    for (uint8_t i = 0U; i < DECODER_COUNT; ++i)
    {
        decoders[i].expected_bytes = MZ_TAPE_HEADER_BYTES;
        reset_candidate(&decoders[i], 0U);
    }
}

void mz_tape_decoder_start_data(uint32_t byte_count, bool invert_pulse_phase)
{
    uint8_t index;

    if (!selected_level_valid || (byte_count > 65535UL))
    {
        decoder_mode = DECODER_MODE_STOPPED;
        return;
    }
    /* The inversion is always relative to the validated header.  This keeps
       repeated block starts deterministic (TC changes physical phase once,
       while NORMAL/IC retain it). */
    index = (uint8_t)(header_start_level ^ (invert_pulse_phase ? 1U : 0U));
    selected_start_level = index;
    decoder_mode = DECODER_MODE_DATA;
    have_previous_half = false;
    previous_half_units = 0U;
    pending_event.type = MZ_TAPE_DECODER_EVENT_NONE;
    decoders[index].expected_bytes = byte_count;
    reset_candidate(&decoders[index], 0U);
}

void mz_tape_decoder_start_recovery_data(uint32_t byte_count)
{
    uint8_t index = selected_start_level;
    if (!selected_level_valid || (byte_count > 65535UL))
    {
        decoder_mode = DECODER_MODE_STOPPED;
        return;
    }
    decoder_mode = DECODER_MODE_DATA;
    have_previous_half = false;
    previous_half_units = 0U;
    pending_event.type = MZ_TAPE_DECODER_EVENT_NONE;
    begin_duplicate_gap(&decoders[index], byte_count);
}

void mz_tape_decoder_break_signal(void)
{
    have_previous_half = false;
    previous_half_units = 0U;
    if (decoder_mode == DECODER_MODE_HEADER)
    {
        for (uint8_t i = 0U; i < DECODER_COUNT; ++i)
        {
            decoders[i].expected_bytes = MZ_TAPE_HEADER_BYTES;
            reset_candidate(&decoders[i], 0U);
        }
    }
    else if ((decoder_mode == DECODER_MODE_DATA) && selected_level_valid)
    {
        reset_candidate(&decoders[selected_start_level], 0U);
    }
}

void mz_tape_decoder_stop(void)
{
    decoder_mode = DECODER_MODE_STOPPED;
    have_previous_half = false;
    pending_event.type = MZ_TAPE_DECODER_EVENT_NONE;
}

void mz_tape_decoder_feed_interval(uint16_t duration_units, uint8_t level)
{
    uint16_t pulse_units;
    uint8_t pair_start_level;

    if (decoder_mode == DECODER_MODE_STOPPED) return;
    level = level ? 1U : 0U;
    if ((duration_units == 0U) || (duration_units > MAX_HALF_UNITS))
    {
        mz_tape_decoder_break_signal();
        return;
    }
    if (!have_previous_half)
    {
        previous_half_units = duration_units;
        previous_half_level = level;
        have_previous_half = true;
        return;
    }
    if (level == previous_half_level)
    {
        mz_tape_decoder_break_signal();
        previous_half_units = duration_units;
        previous_half_level = level;
        have_previous_half = true;
        return;
    }

    pulse_units = (uint16_t)(previous_half_units + duration_units);
    pair_start_level = previous_half_level;
    if ((decoder_mode == DECODER_MODE_HEADER) ||
        (selected_level_valid && (pair_start_level == selected_start_level)))
    {
        feed_pulse(&decoders[pair_start_level], pair_start_level, pulse_units);
    }
    previous_half_units = duration_units;
    previous_half_level = level;
}

bool mz_tape_decoder_take_event(mz_tape_decoder_event_t *event)
{
    if ((event == NULL) ||
        (pending_event.type == MZ_TAPE_DECODER_EVENT_NONE)) return false;
    *event = pending_event;
    pending_event.type = MZ_TAPE_DECODER_EVENT_NONE;
    return true;
}

const uint8_t *mz_tape_decoder_get_header(void)
{
    return validated_header;
}

uint8_t mz_tape_decoder_get_pulse_start_level(void)
{
    return selected_start_level;
}
