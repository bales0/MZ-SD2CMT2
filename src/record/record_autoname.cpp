#include "record_autoname.h"

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "record_path_buffer.h"
#include "../drivers/flash_text.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"

#define AUTONAME_HEADER_BYTES 128U
#define AUTONAME_NAME_BYTES 17U
#define AUTONAME_DECODER_COUNT 2U
/* Fast header variants use a shorter leader and a 20-pulse tape mark;
   standard ROM headers use 40 mark pulses.  The full-header checksum is the
   final guard against a false short-leader match. */
#define AUTONAME_MIN_LEADER_PULSES 16U
#define AUTONAME_MIN_MARK_PULSES 12U
#define AUTONAME_MAX_MARK_PULSES 48U
#define AUTONAME_FINAL_MARK_PULSES 2U
#define AUTONAME_MAX_HALF_UNITS 1024U
#define AUTONAME_MAX_SUFFIX 99U

typedef enum
{
    AUTONAME_SEARCH_LEADER = 0,
    AUTONAME_MARK_LONG,
    AUTONAME_MARK_SHORT,
    AUTONAME_MARK_FINAL,
    AUTONAME_DATA
} autoname_decode_state_t;

typedef struct
{
    autoname_decode_state_t state;
    uint16_t short_x8;
    uint16_t leader_pulses;
    uint8_t mark_pulses;
    uint8_t final_pulses;
    uint8_t bit_count;
    uint8_t byte_value;
    uint8_t byte_index;
    uint16_t checksum;
    uint16_t recorded_checksum;
    uint8_t name[AUTONAME_NAME_BYTES];
} autoname_decoder_t;

static bool autoname_enabled = false;
static bool autoname_found = false;
static autoname_decoder_t autoname_decoders[AUTONAME_DECODER_COUNT];
static char autoname_name[AUTONAME_NAME_BYTES + 1U];

/* Every two adjacent half intervals form a pulse for one of two possible
   phases.  Trying both phases makes the decoder polarity-independent and,
   unlike half-by-half classification, lets 22 kHz WAV and 50 us LEP resolve
   fast asymmetric pulses from their unambiguous full duration. */
static bool autoname_have_previous_half = false;
static uint16_t autoname_previous_half = 0U;
static uint8_t autoname_pair_decoder = 0U;

/* WAV-only run accumulator. Packed source bytes are bit 0 first. */
static bool autoname_sample_active = false;
static uint8_t autoname_sample_level = 0U;
static uint16_t autoname_sample_run = 0U;

/* Complete Sharp MZ-80A character-code to ASCII map. Unsupported graphical
   characters become spaces. Kept in flash so AUTONAME uses no extra SRAM.
   Source: zSoft common/tranzputer.c (GPLv3), Philip Smart. */
static const uint8_t sharp_mz_to_ascii_P[256] PROGMEM = {
    /* 00 */ 0x00U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x00U, 0x20U, 0x20U,
    /* 10 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* 20 */ 0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U, 0x27U,
             0x28U, 0x29U, 0x2AU, 0x2BU, 0x2CU, 0x2DU, 0x2EU, 0x2FU,
    /* 30 */ 0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U,
             0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU, 0x3FU,
    /* 40 */ 0x40U, 0x41U, 0x42U, 0x43U, 0x44U, 0x45U, 0x46U, 0x47U,
             0x48U, 0x49U, 0x4AU, 0x4BU, 0x4CU, 0x4DU, 0x4EU, 0x4FU,
    /* 50 */ 0x50U, 0x51U, 0x52U, 0x53U, 0x54U, 0x55U, 0x56U, 0x57U,
             0x58U, 0x59U, 0x5AU, 0x5BU, 0x5CU, 0x5DU, 0x5EU, 0x5FU,
    /* 60 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* 70 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* 80 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* 90 */ 0x20U, 0x20U, 0x65U, 0x20U, 0x20U, 0x20U, 0x74U, 0x67U,
             0x68U, 0x20U, 0x62U, 0x78U, 0x64U, 0x72U, 0x70U, 0x63U,
    /* A0 */ 0x71U, 0x61U, 0x7AU, 0x77U, 0x73U, 0x75U, 0x69U, 0x20U,
             0x4FU, 0x6BU, 0x66U, 0x76U, 0x20U, 0x75U, 0x42U, 0x6AU,
    /* B0 */ 0x6EU, 0x20U, 0x55U, 0x6DU, 0x20U, 0x20U, 0x20U, 0x6FU,
             0x6CU, 0x41U, 0x6FU, 0x61U, 0x20U, 0x79U, 0x20U, 0x20U,
    /* C0 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* D0 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* E0 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
    /* F0 */ 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U,
             0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U
};

static_assert(sizeof(sharp_mz_to_ascii_P) == 256U,
              "Sharp MZ character map must contain all 256 codes");

static void autoname_decoder_reset(autoname_decoder_t *decoder,
                                   uint16_t seed_units)
{
    if (decoder == NULL)
    {
        return;
    }

    decoder->state = AUTONAME_SEARCH_LEADER;
    decoder->short_x8 =
        (seed_units <= (AUTONAME_MAX_HALF_UNITS * 2U)) ?
            (uint16_t)(seed_units * 8U) : 0U;
    decoder->leader_pulses = (decoder->short_x8 != 0U) ? 1U : 0U;
    decoder->mark_pulses = 0U;
    decoder->final_pulses = 0U;
    decoder->bit_count = 0U;
    decoder->byte_value = 0U;
    decoder->byte_index = 0U;
    decoder->checksum = 0U;
    decoder->recorded_checksum = 0U;
    decoder->name[0] = 0U;
}

static bool autoname_is_leader_pulse(const autoname_decoder_t *decoder,
                                     uint16_t duration_units)
{
    uint32_t scaled = (uint32_t)duration_units * 8UL;
    uint32_t average;
    uint32_t difference;

    if ((decoder == NULL) || (decoder->short_x8 == 0U))
    {
        return false;
    }

    average = decoder->short_x8;
    difference = (scaled >= average) ? (scaled - average) : (average - scaled);

    /* Full periods normally vary by less than 25 %.  One complete input unit
       of extra tolerance is essential for WAV 22 kHz and LEP quantization. */
    return (((scaled * 4UL) >= (average * 3UL)) &&
            ((scaled * 4UL) <= (average * 5UL))) ||
           (difference <= 8UL);
}

/* -1 invalid, 0 short, 1 long. Full-pulse classification remains reliable
   when one fast HIGH or LOW half alone has the same quantized length as a
   half of the other class. The 1.45 midpoint covers the measured 1.75..2.0
   long/short period ratios. */
static int8_t autoname_classify_pulse(const autoname_decoder_t *decoder,
                                      uint16_t duration_units)
{
    uint32_t scaled = (uint32_t)duration_units * 8UL;
    uint32_t average;

    if ((decoder == NULL) || (decoder->short_x8 == 0U))
    {
        return -1;
    }

    average = decoder->short_x8;
    if (((scaled * 2UL) < average) || (scaled > (average * 3UL)))
    {
        return -1;
    }

    return ((scaled * 20UL) < (average * 29UL)) ? 0 : 1;
}

static uint8_t autoname_popcount8(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static bool autoname_safe_ascii(uint8_t value)
{
    return ((value >= 'A') && (value <= 'Z')) ||
           ((value >= 'a') && (value <= 'z')) ||
           ((value >= '0') && (value <= '9')) ||
           (value == ' ') || (value == '-') || (value == '_') ||
           (value == '(') || (value == ')') || (value == '.') ||
           (value == '+');
}

static bool autoname_sanitize(void)
{
    uint8_t input = 0U;
    uint8_t output = 0U;
    bool have_alnum = false;
    bool previous_replacement = false;

    while (input < AUTONAME_NAME_BYTES)
    {
        uint8_t raw_value = (uint8_t)autoname_name[input++];
        uint8_t value;

        if ((raw_value == 0U) || (raw_value == 0x0DU))
        {
            break;
        }
        value = pgm_read_byte(&sharp_mz_to_ascii_P[raw_value]);
        if (((value == ' ') || (value == '.')) && (output == 0U))
        {
            continue;
        }

        if (!autoname_safe_ascii(value))
        {
            if (previous_replacement)
            {
                continue;
            }
            value = '_';
            previous_replacement = true;
        }
        else
        {
            previous_replacement = false;
        }

        if (((value >= 'A') && (value <= 'Z')) ||
            ((value >= 'a') && (value <= 'z')) ||
            ((value >= '0') && (value <= '9')))
        {
            have_alnum = true;
        }
        autoname_name[output++] = (char)value;
    }

    while ((output != 0U) &&
           ((autoname_name[output - 1U] == ' ') ||
            (autoname_name[output - 1U] == '.')))
    {
        output--;
    }
    autoname_name[output] = '\0';
    return have_alnum && (output != 0U);
}

static void autoname_accept_byte(autoname_decoder_t *decoder, uint8_t value)
{
    if (decoder->byte_index < AUTONAME_HEADER_BYTES)
    {
        decoder->checksum = (uint16_t)(decoder->checksum +
                                       autoname_popcount8(value));
        if ((decoder->byte_index >= 1U) &&
            (decoder->byte_index <= AUTONAME_NAME_BYTES))
        {
            decoder->name[decoder->byte_index - 1U] = value;
        }
    }
    else
    {
        decoder->recorded_checksum =
            (uint16_t)((decoder->recorded_checksum << 8U) | value);
    }

    decoder->byte_index++;
    if (decoder->byte_index == (AUTONAME_HEADER_BYTES + 2U))
    {
        if (decoder->recorded_checksum == decoder->checksum)
        {
            memcpy(autoname_name, decoder->name, AUTONAME_NAME_BYTES);
            autoname_name[AUTONAME_NAME_BYTES] = '\0';
            if (autoname_sanitize())
            {
                autoname_found = true;
                return;
            }
        }
        autoname_decoder_reset(decoder, 0U);
    }
}

static void autoname_accept_data_pulse(autoname_decoder_t *decoder,
                                       uint8_t pulse_class)
{
    if (decoder->bit_count < 8U)
    {
        decoder->byte_value =
            (uint8_t)((decoder->byte_value << 1U) | pulse_class);
        decoder->bit_count++;
        return;
    }

    /* Every MZ header byte, including the two checksum bytes, ends in one
       long stop pulse. */
    if (pulse_class != 1U)
    {
        autoname_decoder_reset(decoder, 0U);
        return;
    }

    autoname_accept_byte(decoder, decoder->byte_value);
    decoder->byte_value = 0U;
    decoder->bit_count = 0U;
}

static void autoname_feed_pulse(autoname_decoder_t *decoder,
                                uint16_t duration_units)
{
    int8_t pulse_class;

    if ((decoder == NULL) || autoname_found)
    {
        return;
    }

    if (decoder->state == AUTONAME_SEARCH_LEADER)
    {
        uint32_t scaled;

        if (decoder->short_x8 == 0U)
        {
            autoname_decoder_reset(decoder, duration_units);
            return;
        }

        scaled = (uint32_t)duration_units * 8UL;
        if (autoname_is_leader_pulse(decoder, duration_units))
        {
            decoder->short_x8 = (uint16_t)
                ((((uint32_t)decoder->short_x8 * 7UL) + scaled + 4UL) / 8UL);
            if (decoder->leader_pulses != 0xFFFFU)
            {
                decoder->leader_pulses++;
            }
            return;
        }

        pulse_class = autoname_classify_pulse(decoder, duration_units);
        if ((decoder->leader_pulses >= AUTONAME_MIN_LEADER_PULSES) &&
            (pulse_class == 1))
        {
            decoder->state = AUTONAME_MARK_LONG;
            decoder->mark_pulses = 1U;
            return;
        }

        autoname_decoder_reset(decoder, duration_units);
        return;
    }

    pulse_class = autoname_classify_pulse(decoder, duration_units);
    if (pulse_class < 0)
    {
        autoname_decoder_reset(decoder, duration_units);
        return;
    }

    if (decoder->state == AUTONAME_MARK_LONG)
    {
        if (pulse_class == 1)
        {
            if (decoder->mark_pulses != 0xFFU) decoder->mark_pulses++;
            return;
        }
        if ((decoder->mark_pulses >= AUTONAME_MIN_MARK_PULSES) &&
            (decoder->mark_pulses <= AUTONAME_MAX_MARK_PULSES))
        {
            decoder->state = AUTONAME_MARK_SHORT;
            decoder->mark_pulses = 1U;
            return;
        }
        autoname_decoder_reset(decoder, duration_units);
        return;
    }

    if (decoder->state == AUTONAME_MARK_SHORT)
    {
        if (pulse_class == 0)
        {
            if (decoder->mark_pulses != 0xFFU) decoder->mark_pulses++;
            return;
        }
        if ((decoder->mark_pulses >= AUTONAME_MIN_MARK_PULSES) &&
            (decoder->mark_pulses <= AUTONAME_MAX_MARK_PULSES))
        {
            decoder->state = AUTONAME_MARK_FINAL;
            decoder->final_pulses = 1U;
            return;
        }
        autoname_decoder_reset(decoder, duration_units);
        return;
    }

    if (decoder->state == AUTONAME_MARK_FINAL)
    {
        if (pulse_class != 1)
        {
            autoname_decoder_reset(decoder, duration_units);
            return;
        }
        decoder->final_pulses++;
        if (decoder->final_pulses == AUTONAME_FINAL_MARK_PULSES)
        {
            decoder->state = AUTONAME_DATA;
        }
        return;
    }

    autoname_accept_data_pulse(decoder, (uint8_t)pulse_class);
}

void record_autoname_feed_interval(uint16_t duration_units)
{
    uint16_t pulse_units;

    if (!autoname_enabled || autoname_found)
    {
        return;
    }
    if ((duration_units == 0U) ||
        (duration_units > AUTONAME_MAX_HALF_UNITS))
    {
        record_autoname_break_signal();
        return;
    }

    if (!autoname_have_previous_half)
    {
        autoname_previous_half = duration_units;
        autoname_have_previous_half = true;
        autoname_pair_decoder = 0U;
        return;
    }

    pulse_units = (uint16_t)(autoname_previous_half + duration_units);
    autoname_feed_pulse(&autoname_decoders[autoname_pair_decoder],
                        pulse_units);
    autoname_pair_decoder ^= 1U;
    autoname_previous_half = duration_units;
}

void record_autoname_begin(bool enabled)
{
    autoname_enabled = enabled;
    autoname_found = false;
    autoname_sample_active = false;
    autoname_sample_level = 0U;
    autoname_sample_run = 0U;
    autoname_have_previous_half = false;
    autoname_previous_half = 0U;
    autoname_pair_decoder = 0U;
    autoname_name[0] = '\0';
    for (uint8_t i = 0U; i < AUTONAME_DECODER_COUNT; ++i)
    {
        autoname_decoder_reset(&autoname_decoders[i], 0U);
    }
}

void record_autoname_break_signal(void)
{
    if (!autoname_enabled || autoname_found)
    {
        return;
    }
    autoname_sample_active = false;
    autoname_sample_run = 0U;
    autoname_have_previous_half = false;
    autoname_previous_half = 0U;
    autoname_pair_decoder = 0U;
    for (uint8_t i = 0U; i < AUTONAME_DECODER_COUNT; ++i)
    {
        autoname_decoder_reset(&autoname_decoders[i], 0U);
    }
}

void record_autoname_feed_packed_samples(uint8_t packed, uint8_t valid_bits)
{
    uint8_t bit;

    if (!autoname_enabled || autoname_found)
    {
        return;
    }
    if (valid_bits > 8U)
    {
        valid_bits = 8U;
    }

    for (bit = 0U; bit < valid_bits; ++bit)
    {
        uint8_t level = (packed & (uint8_t)(1U << bit)) ? 1U : 0U;

        if (!autoname_sample_active)
        {
            autoname_sample_active = true;
            autoname_sample_level = level;
            autoname_sample_run = 1U;
        }
        else if (level == autoname_sample_level)
        {
            if (autoname_sample_run != 0xFFFFU) autoname_sample_run++;
        }
        else
        {
            record_autoname_feed_interval(autoname_sample_run);
            autoname_sample_level = level;
            autoname_sample_run = 1U;
        }
    }
}

bool record_autoname_has_name(void)
{
    return autoname_enabled && autoname_found;
}

const char *record_autoname_get_name(void)
{
    return record_autoname_has_name() ? autoname_name : NULL;
}

static int autoname_make_path(char *destination,
                              const char *directory_path,
                              file_format_t format,
                              uint8_t suffix)
{
    if (suffix == 0U)
    {
        if (format == FILE_FORMAT_WAV)
            return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                                       PSTR("%s/%s.WAV"), directory_path,
                                       autoname_name);
        if (format == FILE_FORMAT_L16)
            return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                                       PSTR("%s/%s.L16"), directory_path,
                                       autoname_name);
        return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                                   PSTR("%s/%s.LEP"), directory_path,
                                   autoname_name);
    }

    if (format == FILE_FORMAT_WAV)
        return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                                   PSTR("%s/%s_%02u.WAV"), directory_path,
                                   autoname_name, (unsigned int)suffix);
    if (format == FILE_FORMAT_L16)
        return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                                   PSTR("%s/%s_%02u.L16"), directory_path,
                                   autoname_name, (unsigned int)suffix);
    return flash_text_snprintf(destination, RECORD_PATH_BUFFER_MAX,
                               PSTR("%s/%s_%02u.LEP"), directory_path,
                               autoname_name, (unsigned int)suffix);
}

bool record_autoname_apply(const char *directory_path, file_format_t format)
{
    char *target = (char *)wav_sample_stream_get_shared_work_buffer();
    uint8_t suffix;

    if (!record_autoname_has_name() || (directory_path == NULL) ||
        ((format != FILE_FORMAT_WAV) && (format != FILE_FORMAT_L16) &&
         (format != FILE_FORMAT_LEP)) || sdcard_file_is_open())
    {
        return false;
    }

    for (suffix = 0U; suffix <= AUTONAME_MAX_SUFFIX; ++suffix)
    {
        int length = autoname_make_path(target, directory_path, format, suffix);

        if ((length <= 0) || (length >= (int)RECORD_PATH_BUFFER_MAX))
        {
            return false;
        }
        if (strcmp(target, record_path_buffer) == 0)
        {
            return true;
        }
        if (sdcard_file_exists(target))
        {
            continue;
        }
        if (!sdcard_file_rename(record_path_buffer, target))
        {
            return false;
        }

        strncpy(record_path_buffer, target, RECORD_PATH_BUFFER_MAX - 1U);
        record_path_buffer[RECORD_PATH_BUFFER_MAX - 1U] = '\0';
        return true;
    }

    return false;
}
