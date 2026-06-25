#include "edge_record_engine.h"

#include <stdio.h>
#include <string.h>

#include "edge_record_driver.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"
#include "../streams/cmt_mode_scratch.h"

#define EDGE_RECORD_PATH_MAX 160U
#define EDGE_RECORD_FILENAME_MAX 16U
#define EDGE_RECORD_STAGE_BYTES 512U
#define EDGE_RECORD_FINAL_PREALLOCATE_BYTES (2UL * 1024UL * 1024UL)

#define edge_stage0_buffer wav_sample_stream_get_shared_work_buffer()
#define edge_stage1_buffer cmt_mode_scratch.edge_record_stage_bytes

typedef enum
{
    EDGE_PARSE_NORMAL = 0,
    EDGE_PARSE_UNIT,
    EDGE_PARSE_LONG
} edge_parse_state_t;

static edge_record_engine_state_t record_state = EDGE_RECORD_ENGINE_STOPPED;
static file_format_t record_format = FILE_FORMAT_UNKNOWN;
static char record_full_path[EDGE_RECORD_PATH_MAX];
static char record_filename[EDGE_RECORD_FILENAME_MAX];
static char record_error_text[17];

/* Two 512-byte standard-output sectors. Stage 1 reuses browser-only RAM. */
static uint16_t stage_count[2];
static bool stage_ready[2];
static uint8_t stage_fill_index = 0U;
static uint8_t stage_write_index = 0U;
static uint32_t final_bytes_written = 0UL;

/* Driver token parser. */
static edge_parse_state_t parse_state = EDGE_PARSE_NORMAL;
static uint8_t parse_bytes_needed = 0U;
static uint8_t parse_shift = 0U;
static uint32_t parse_long_units = 0UL;
static uint8_t parse_pending_short_units = 0U;

/* Standard LEP/L16 stream emitter. */
static uint8_t output_level = 0U;
static bool output_interval_active = false;
static bool output_first_pending = false;
static uint32_t output_zero_count = 0UL;
static uint32_t output_interval_units = 0UL;
static uint8_t output_interval_level = 0U;

static uint8_t *edge_stage_buffer(uint8_t index)
{
    return (index == 0U) ? edge_stage0_buffer : edge_stage1_buffer;
}

static void edge_record_set_error(const char *text)
{
    if (text == NULL)
    {
        text = "EDGE ERROR";
    }
    strncpy(record_error_text, text, sizeof(record_error_text) - 1U);
    record_error_text[sizeof(record_error_text) - 1U] = '\0';
}

static const char *edge_record_extension(void)
{
    return (record_format == FILE_FORMAT_L16) ? "L16" : "LEP";
}

static bool edge_record_make_paths(const char *directory_path)
{
    uint16_t sequence;
    int length;

    if (!sdcard_next_record_sequence(directory_path, &sequence))
    {
        edge_record_set_error(sdcard_last_error());
        return false;
    }

    length = snprintf(record_full_path, sizeof(record_full_path),
                      "%s/REC%04u.%s", directory_path,
                      (unsigned int)sequence, edge_record_extension());

    if ((length <= 0) || ((size_t)length >= sizeof(record_full_path)))
    {
        edge_record_set_error("PATH TOO LONG");
        return false;
    }

    snprintf(record_filename, sizeof(record_filename), "REC%04u.%s",
             (unsigned int)sequence, edge_record_extension());
    return true;
}

/* Submit only one completed sector. SD writes never run in an ISR. */
static bool edge_record_write_one_ready_stage(void)
{
    uint8_t index = stage_write_index;
    uint16_t count;

    if (!stage_ready[index])
    {
        return true;
    }
    if (sdcard_file_is_busy())
    {
        return true;
    }

    count = stage_count[index];
    if ((count == 0U) || (count > EDGE_RECORD_STAGE_BYTES))
    {
        edge_record_set_error("STAGE ERROR");
        return false;
    }

    if (sdcard_file_write(edge_stage_buffer(index), count) != (int16_t)count)
    {
        edge_record_set_error("EDGE WRITE");
        return false;
    }

    final_bytes_written += (uint32_t)count;
    stage_count[index] = 0U;
    stage_ready[index] = false;
    stage_write_index ^= 1U;
    return true;
}

static bool edge_record_emit_byte(uint8_t value)
{
    uint8_t index = stage_fill_index;

    if (stage_ready[index])
    {
        return false;
    }

    edge_stage_buffer(index)[stage_count[index]++] = value;
    if (stage_count[index] == EDGE_RECORD_STAGE_BYTES)
    {
        stage_ready[index] = true;
        stage_fill_index ^= 1U;
    }
    return true;
}

/* Start expansion of one full logical level interval. */
static bool edge_record_begin_interval(uint32_t units)
{
    if (units == 0UL)
    {
        edge_record_set_error("BAD TOKEN");
        return false;
    }

    output_interval_units = units;
    output_interval_level = output_level;
    output_level ^= 1U;
    output_first_pending = true;
    output_zero_count = 0UL;
    output_interval_active = true;
    return true;
}

/*
   Write valid LEP/L16 bytes directly to the final file staging area:
     duration N = signed remainder 1..127 followed by zero extensions.
*/
static bool edge_record_emit_active_interval(void)
{
    while (output_interval_active)
    {
        if (output_first_pending)
        {
            uint32_t zero_count = (output_interval_units - 1UL) / 127UL;
            uint32_t first_units = output_interval_units - zero_count * 127UL;
            int8_t first = output_interval_level ? (int8_t)first_units :
                (int8_t)(-(int16_t)first_units);

            if (!edge_record_emit_byte((uint8_t)first))
            {
                return true;
            }

            output_zero_count = zero_count;
            output_first_pending = false;
            if (output_zero_count == 0UL)
            {
                output_interval_active = false;
            }
            continue;
        }

        if (!edge_record_emit_byte(0U))
        {
            return true;
        }

        output_zero_count--;
        if (output_zero_count == 0UL)
        {
            output_interval_active = false;
        }
    }
    return true;
}

/* Consume driver tokens and immediately form standard output bytes. */
static bool edge_record_pump_fifo_to_standard_output(void)
{
    while (!stage_ready[stage_fill_index])
    {
        uint8_t value;

        if (output_interval_active)
        {
            if (!edge_record_emit_active_interval())
            {
                return false;
            }
            if (output_interval_active)
            {
                return true;
            }
            continue;
        }

        if (parse_pending_short_units != 0U)
        {
            uint8_t units = parse_pending_short_units;
            parse_pending_short_units = 0U;
            if (!edge_record_begin_interval((uint32_t)units))
            {
                return false;
            }
            continue;
        }

        if (!edge_record_driver_pop_byte(&value))
        {
            return true;
        }

        if (parse_state == EDGE_PARSE_UNIT)
        {
            parse_state = EDGE_PARSE_NORMAL;
            if ((value < 16U) || (value > 127U) ||
                !edge_record_begin_interval((uint32_t)value))
            {
                edge_record_set_error("BAD TOKEN");
                return false;
            }
            continue;
        }

        if (parse_state == EDGE_PARSE_LONG)
        {
            parse_long_units |= ((uint32_t)value << parse_shift);
            parse_shift = (uint8_t)(parse_shift + 8U);
            parse_bytes_needed--;
            if (parse_bytes_needed == 0U)
            {
                parse_state = EDGE_PARSE_NORMAL;
                if (!edge_record_begin_interval(parse_long_units))
                {
                    return false;
                }
            }
            continue;
        }

        if (value == EDGE_RECORD_TOKEN_UNIT)
        {
            parse_state = EDGE_PARSE_UNIT;
            continue;
        }

        if (value == EDGE_RECORD_TOKEN_LONG)
        {
            parse_state = EDGE_PARSE_LONG;
            parse_bytes_needed = 4U;
            parse_shift = 0U;
            parse_long_units = 0UL;
            continue;
        }

        {
            uint8_t first = (uint8_t)(value >> 4);
            uint8_t second = (uint8_t)(value & 0x0FU);

            if (first == 0U)
            {
                edge_record_set_error("BAD TOKEN");
                return false;
            }

            if (!edge_record_begin_interval((uint32_t)first))
            {
                return false;
            }
            if (second != 0U)
            {
                parse_pending_short_units = second;
            }
        }
    }

    return true;
}

static bool edge_record_output_drained(void)
{
    return (edge_record_driver_available_bytes() == 0U) &&
           (parse_state == EDGE_PARSE_NORMAL) &&
           (parse_pending_short_units == 0U) &&
           !output_interval_active;
}

static void edge_record_mark_output_tail_ready(void)
{
    uint8_t index = stage_fill_index;

    if ((stage_count[index] != 0U) && !stage_ready[index])
    {
        stage_ready[index] = true;
        stage_fill_index ^= 1U;
    }
}

static void edge_record_fail_close(const char *text)
{
    if (text != NULL)
    {
        edge_record_set_error(text);
    }

    edge_record_driver_abort();
    sdcard_file_close();
    if (record_full_path[0] != '\0')
    {
        (void)sdcard_file_remove(record_full_path);
    }
    record_state = EDGE_RECORD_ENGINE_ERROR;
}

void edge_record_engine_init(void)
{
    edge_record_driver_init();
    record_state = EDGE_RECORD_ENGINE_STOPPED;
    record_format = FILE_FORMAT_UNKNOWN;
    record_full_path[0] = '\0';
    record_filename[0] = '\0';
    record_error_text[0] = '\0';
    stage_count[0] = 0U;
    stage_count[1] = 0U;
    stage_ready[0] = false;
    stage_ready[1] = false;
    stage_fill_index = 0U;
    stage_write_index = 0U;
    final_bytes_written = 0UL;
    parse_state = EDGE_PARSE_NORMAL;
    parse_bytes_needed = 0U;
    parse_shift = 0U;
    parse_long_units = 0UL;
    parse_pending_short_units = 0U;
    output_level = 0U;
    output_interval_active = false;
    output_first_pending = false;
    output_zero_count = 0UL;
    output_interval_units = 0UL;
    output_interval_level = 0U;
}

bool edge_record_engine_preview_filename(const char *directory_path,
                                         file_format_t format)
{
    if ((format != FILE_FORMAT_LEP) && (format != FILE_FORMAT_L16))
    {
        edge_record_set_error("REC FORMAT");
        return false;
    }

    record_format = format;
    return edge_record_make_paths(directory_path);
}

bool edge_record_engine_start(const char *directory_path, file_format_t format)
{
    uint8_t unit_us;

    edge_record_engine_init();

    if ((format != FILE_FORMAT_LEP) && (format != FILE_FORMAT_L16))
    {
        edge_record_set_error("EDGE FORMAT");
        record_state = EDGE_RECORD_ENGINE_ERROR;
        return false;
    }
    if (!sdcard_is_mounted())
    {
        edge_record_set_error("SD CARD ERROR");
        record_state = EDGE_RECORD_ENGINE_ERROR;
        return false;
    }

    record_format = format;
    unit_us = (format == FILE_FORMAT_L16) ? 16U : 50U;

    if (!edge_record_driver_prepare(unit_us) ||
        !edge_record_make_paths(directory_path))
    {
        record_state = EDGE_RECORD_ENGINE_ERROR;
        return false;
    }
    if (!sdcard_file_open_write(record_full_path))
    {
        edge_record_set_error("EDGE CREATE");
        record_state = EDGE_RECORD_ENGINE_ERROR;
        return false;
    }

    /* Preallocate the final standard file before PCINT capture begins. */
    if (!sdcard_file_preallocate(EDGE_RECORD_FINAL_PREALLOCATE_BYTES) ||
        !sdcard_file_seek(0UL) || !sdcard_file_sync())
    {
        edge_record_fail_close("PREALLOC FAIL");
        return false;
    }

    if (!edge_record_driver_start())
    {
        edge_record_fail_close("EDGE START");
        return false;
    }

    output_level = edge_record_driver_get_initial_level();
    record_state = EDGE_RECORD_ENGINE_RECORDING;
    return true;
}

bool edge_record_engine_pause(void)
{
    if ((record_state == EDGE_RECORD_ENGINE_RECORDING) &&
        edge_record_driver_pause())
    {
        record_state = EDGE_RECORD_ENGINE_PAUSED;
        return true;
    }
    return false;
}

bool edge_record_engine_resume(void)
{
    if ((record_state == EDGE_RECORD_ENGINE_PAUSED) &&
        edge_record_driver_resume())
    {
        record_state = EDGE_RECORD_ENGINE_RECORDING;
        return true;
    }
    return false;
}

void edge_record_engine_request_stop(void)
{
    if ((record_state != EDGE_RECORD_ENGINE_RECORDING) &&
        (record_state != EDGE_RECORD_ENGINE_PAUSED))
    {
        return;
    }

    edge_record_driver_stop();
    if (edge_record_driver_get_state() == EDGE_RECORD_DRIVER_OVERRUN)
    {
        edge_record_fail_close("EDGE OVERFLOW");
        return;
    }
    if (edge_record_driver_get_state() == EDGE_RECORD_DRIVER_TOO_LONG)
    {
        edge_record_fail_close("EDGE TOO LONG");
        return;
    }

    record_state = EDGE_RECORD_ENGINE_FINALIZING;
}

void edge_record_engine_cancel(void)
{
    edge_record_driver_abort();
    sdcard_file_close();
    if (record_full_path[0] != '\0')
    {
        (void)sdcard_file_remove(record_full_path);
    }
    record_state = EDGE_RECORD_ENGINE_STOPPED;
}

void edge_record_engine_service(void)
{
    edge_record_driver_state_t driver_state = edge_record_driver_get_state();

    if ((record_state == EDGE_RECORD_ENGINE_RECORDING) ||
        (record_state == EDGE_RECORD_ENGINE_PAUSED))
    {
        if (driver_state == EDGE_RECORD_DRIVER_OVERRUN)
        {
            edge_record_fail_close("EDGE OVERFLOW");
            return;
        }
        if (driver_state == EDGE_RECORD_DRIVER_TOO_LONG)
        {
            edge_record_fail_close("EDGE TOO LONG");
            return;
        }

        if (!edge_record_write_one_ready_stage() ||
            !edge_record_pump_fifo_to_standard_output())
        {
            edge_record_fail_close(record_error_text);
        }
        return;
    }

    if (record_state != EDGE_RECORD_ENGINE_FINALIZING)
    {
        return;
    }

    if (!edge_record_write_one_ready_stage() ||
        !edge_record_pump_fifo_to_standard_output())
    {
        edge_record_fail_close(record_error_text);
        return;
    }

    if (!edge_record_output_drained())
    {
        return;
    }

    edge_record_mark_output_tail_ready();
    if (!edge_record_write_one_ready_stage())
    {
        edge_record_fail_close(record_error_text);
        return;
    }

    if (stage_ready[0] || stage_ready[1] ||
        (stage_count[0] != 0U) || (stage_count[1] != 0U) ||
        sdcard_file_is_busy())
    {
        return;
    }

    if (!sdcard_file_truncate(final_bytes_written) || !sdcard_file_sync())
    {
        edge_record_fail_close("EDGE CLOSE");
        return;
    }

    sdcard_file_close();
    record_state = EDGE_RECORD_ENGINE_FINISHED;
}

edge_record_engine_state_t edge_record_engine_get_state(void)
{
    return record_state;
}

const char *edge_record_engine_get_filename(void)
{
    return record_filename;
}

const char *edge_record_engine_get_full_path(void)
{
    return record_full_path;
}

const char *edge_record_engine_get_error_text(void)
{
    return record_error_text;
}

uint8_t edge_record_engine_get_buffer_fill_percent(void)
{
    return edge_record_driver_fill_percent();
}

uint8_t edge_record_engine_get_buffer_headroom_percent(void)
{
    const uint32_t capacity = (uint32_t)(WAV_SAMPLE_STREAM_BUFFER_BYTES - 1U) +
                              (2UL * EDGE_RECORD_STAGE_BYTES);
    uint32_t used = (uint32_t)edge_record_driver_available_bytes() +
                    (uint32_t)stage_count[0] + (uint32_t)stage_count[1];

    if (used > capacity)
    {
        used = capacity;
    }
    return (uint8_t)(((capacity - used) * 100UL) / capacity);
}
