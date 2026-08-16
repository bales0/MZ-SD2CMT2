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
#define AUTONAME_MIN_LEADER_HALVES 128U
#define AUTONAME_MIN_MARK_HALVES 60U
#define AUTONAME_MAX_MARK_HALVES 100U
#define AUTONAME_FINAL_MARK_HALVES 4U
#define AUTONAME_MAX_INTERVAL_UNITS 1024U
#define AUTONAME_MAX_SUFFIX 99U

typedef enum
{
    AUTONAME_SEARCH_LEADER = 0,
    AUTONAME_MARK_LONG,
    AUTONAME_MARK_SHORT,
    AUTONAME_MARK_FINAL,
    AUTONAME_DATA
} autoname_decode_state_t;

static bool autoname_enabled = false;
static bool autoname_found = false;
static autoname_decode_state_t autoname_state = AUTONAME_SEARCH_LEADER;

/* The unit is deliberately arbitrary: WAV uses samples, L16/LEP their native
   quantization. The leader derives the short half-period for this recording. */
static uint16_t autoname_short_x8 = 0U;
static uint16_t autoname_leader_halves = 0U;
static uint8_t autoname_mark_halves = 0U;
static uint8_t autoname_final_halves = 0U;

static uint8_t autoname_pending_half = 0U;
static uint8_t autoname_bit_count = 0U;
static uint8_t autoname_byte_value = 0U;
static uint8_t autoname_byte_index = 0U;
static uint16_t autoname_checksum = 0U;
static uint16_t autoname_recorded_checksum = 0U;
static char autoname_name[AUTONAME_NAME_BYTES + 1U];

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

static void autoname_reset_search(uint16_t seed_units)
{
    autoname_state = AUTONAME_SEARCH_LEADER;
    autoname_short_x8 = ((seed_units >= 2U) &&
                         (seed_units <= AUTONAME_MAX_INTERVAL_UNITS)) ?
        (uint16_t)(seed_units * 8U) : 0U;
    autoname_leader_halves = (autoname_short_x8 != 0U) ? 1U : 0U;
    autoname_mark_halves = 0U;
    autoname_final_halves = 0U;
    autoname_pending_half = 0U;
    autoname_bit_count = 0U;
    autoname_byte_value = 0U;
    autoname_byte_index = 0U;
    autoname_checksum = 0U;
    autoname_recorded_checksum = 0U;
    autoname_name[0] = '\0';
}

/* -1 invalid, 0 short, 1 long. The midpoint is 1.5 times the calibrated
   short half-period; the broad outer limits tolerate PCM quantization. */
static int8_t autoname_classify_half(uint16_t duration_units)
{
    uint32_t scaled = (uint32_t)duration_units * 8UL;

    if ((autoname_short_x8 == 0U) ||
        ((scaled * 4UL) < (uint32_t)autoname_short_x8) ||
        (scaled > ((uint32_t)autoname_short_x8 * 3UL)))
    {
        return -1;
    }

    return ((scaled * 2UL) < ((uint32_t)autoname_short_x8 * 3UL)) ? 0 : 1;
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

static void autoname_accept_byte(uint8_t value)
{
    if (autoname_byte_index < AUTONAME_HEADER_BYTES)
    {
        autoname_checksum = (uint16_t)(autoname_checksum +
                                      autoname_popcount8(value));
        if ((autoname_byte_index >= 1U) &&
            (autoname_byte_index <= AUTONAME_NAME_BYTES))
        {
            autoname_name[autoname_byte_index - 1U] = (char)value;
        }
    }
    else
    {
        autoname_recorded_checksum =
            (uint16_t)((autoname_recorded_checksum << 8U) | value);
    }

    autoname_byte_index++;
    if (autoname_byte_index == (AUTONAME_HEADER_BYTES + 2U))
    {
        autoname_name[AUTONAME_NAME_BYTES] = '\0';
        if ((autoname_recorded_checksum == autoname_checksum) &&
            autoname_sanitize())
        {
            autoname_found = true;
            return;
        }
        autoname_reset_search(0U);
    }
}

static void autoname_accept_pulse(uint8_t pulse_class)
{
    if (autoname_bit_count < 8U)
    {
        autoname_byte_value =
            (uint8_t)((autoname_byte_value << 1U) | pulse_class);
        autoname_bit_count++;
        return;
    }

    /* Every MZ header byte, including the two checksum bytes, ends in one
       long stop pulse. */
    if (pulse_class != 1U)
    {
        autoname_reset_search(0U);
        return;
    }

    autoname_accept_byte(autoname_byte_value);
    autoname_byte_value = 0U;
    autoname_bit_count = 0U;
}

void record_autoname_feed_interval(uint16_t duration_units)
{
    int8_t pulse_class;

    if (!autoname_enabled || autoname_found)
    {
        return;
    }
    if ((duration_units < 2U) ||
        (duration_units > AUTONAME_MAX_INTERVAL_UNITS))
    {
        autoname_reset_search(0U);
        return;
    }

    if (autoname_state == AUTONAME_SEARCH_LEADER)
    {
        uint32_t scaled;

        if (autoname_short_x8 == 0U)
        {
            autoname_reset_search(duration_units);
            return;
        }

        scaled = (uint32_t)duration_units * 8UL;
        if (((scaled * 2UL) >= (uint32_t)autoname_short_x8) &&
            ((scaled * 2UL) <= ((uint32_t)autoname_short_x8 * 3UL)))
        {
            autoname_short_x8 = (uint16_t)
                ((((uint32_t)autoname_short_x8 * 7UL) + scaled + 4UL) / 8UL);
            if (autoname_leader_halves != 0xFFFFU)
            {
                autoname_leader_halves++;
            }
            return;
        }

        pulse_class = autoname_classify_half(duration_units);
        if ((autoname_leader_halves >= AUTONAME_MIN_LEADER_HALVES) &&
            (pulse_class == 1))
        {
            autoname_state = AUTONAME_MARK_LONG;
            autoname_mark_halves = 1U;
            return;
        }

        autoname_reset_search(duration_units);
        return;
    }

    pulse_class = autoname_classify_half(duration_units);
    if (pulse_class < 0)
    {
        autoname_reset_search(duration_units);
        return;
    }

    if (autoname_state == AUTONAME_MARK_LONG)
    {
        if (pulse_class == 1)
        {
            if (autoname_mark_halves != 0xFFU) autoname_mark_halves++;
            return;
        }
        if ((autoname_mark_halves >= AUTONAME_MIN_MARK_HALVES) &&
            (autoname_mark_halves <= AUTONAME_MAX_MARK_HALVES))
        {
            autoname_state = AUTONAME_MARK_SHORT;
            autoname_mark_halves = 1U;
            return;
        }
        autoname_reset_search(duration_units);
        return;
    }

    if (autoname_state == AUTONAME_MARK_SHORT)
    {
        if (pulse_class == 0)
        {
            if (autoname_mark_halves != 0xFFU) autoname_mark_halves++;
            return;
        }
        if ((autoname_mark_halves >= AUTONAME_MIN_MARK_HALVES) &&
            (autoname_mark_halves <= AUTONAME_MAX_MARK_HALVES))
        {
            autoname_state = AUTONAME_MARK_FINAL;
            autoname_final_halves = 1U;
            return;
        }
        autoname_reset_search(duration_units);
        return;
    }

    if (autoname_state == AUTONAME_MARK_FINAL)
    {
        if (pulse_class != 1)
        {
            autoname_reset_search(duration_units);
            return;
        }
        autoname_final_halves++;
        if (autoname_final_halves == AUTONAME_FINAL_MARK_HALVES)
        {
            autoname_state = AUTONAME_DATA;
            autoname_pending_half = 0U;
        }
        return;
    }

    /* A complete pulse must contain two halves of the same class. Zero means
       no pending half, so store classes as 1=short and 2=long. */
    pulse_class++;
    if (autoname_pending_half == 0U)
    {
        autoname_pending_half = (uint8_t)pulse_class;
        return;
    }
    if (autoname_pending_half != (uint8_t)pulse_class)
    {
        autoname_reset_search(duration_units);
        return;
    }

    autoname_pending_half = 0U;
    autoname_accept_pulse((uint8_t)(pulse_class - 1));
}

void record_autoname_begin(bool enabled)
{
    autoname_enabled = enabled;
    autoname_found = false;
    autoname_sample_active = false;
    autoname_sample_level = 0U;
    autoname_sample_run = 0U;
    autoname_reset_search(0U);
}

void record_autoname_break_signal(void)
{
    if (!autoname_enabled || autoname_found)
    {
        return;
    }
    autoname_sample_active = false;
    autoname_sample_run = 0U;
    autoname_reset_search(0U);
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
