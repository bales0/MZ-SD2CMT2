#include "edge_record_engine.h"

#include <stdio.h>
#include <string.h>

#include "edge_record_driver.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"

#define EDGE_RECORD_PATH_MAX 160U
#define EDGE_RECORD_FILENAME_MAX 16U
#define EDGE_RECORD_WRITE_BLOCK 512U

static edge_record_engine_state_t record_state = EDGE_RECORD_ENGINE_STOPPED;
static file_format_t record_format = FILE_FORMAT_UNKNOWN;
static char record_full_path[EDGE_RECORD_PATH_MAX];
static char record_filename[EDGE_RECORD_FILENAME_MAX];
static char record_error_text[17];
static uint16_t work_count = 0U;

static bool long_pending = false;
static uint8_t long_level = 0U;
static uint32_t long_units_remaining = 0UL;
static bool long_first_pending = false;

#define edge_work_buffer wav_sample_stream_get_shared_work_buffer()

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
    if ((directory_path == NULL) || (directory_path[0] == '\0'))
    {
        edge_record_set_error("BAD DIRECTORY");
        return false;
    }

    for (uint16_t index = 1U; index <= 9999U; ++index)
    {
        int length;
        if (strcmp(directory_path, "/") == 0)
        {
            length = snprintf(record_full_path, sizeof(record_full_path),
                              "/REC%04u.%s", (unsigned int)index,
                              edge_record_extension());
        }
        else
        {
            length = snprintf(record_full_path, sizeof(record_full_path),
                              "%s/REC%04u.%s", directory_path,
                              (unsigned int)index, edge_record_extension());
        }

        if ((length <= 0) || ((size_t)length >= sizeof(record_full_path)))
        {
            edge_record_set_error("PATH TOO LONG");
            return false;
        }

        if (!sdcard_file_exists(record_full_path))
        {
            snprintf(record_filename, sizeof(record_filename), "REC%04u.%s",
                     (unsigned int)index, edge_record_extension());
            return true;
        }
    }

    edge_record_set_error("REC NAMES FULL");
    return false;
}

static bool edge_record_flush_work(bool allow_partial)
{
    if (work_count == 0U)
    {
        return true;
    }

    /* During active capture write complete sectors only. Finalization may
       also write the last partial block. Both paths must wait for the SD
       card instead of entering a blocking write while it is programming. */
    if (!allow_partial && (work_count < EDGE_RECORD_WRITE_BLOCK))
    {
        return true;
    }

    if (sdcard_file_is_busy())
    {
        return true;
    }

    if (sdcard_file_write(edge_work_buffer, work_count) != (int16_t)work_count)
    {
        edge_record_set_error("EDGE WRITE");
        return false;
    }

    work_count = 0U;
    return true;
}

static bool edge_record_emit_file_byte(uint8_t value)
{
    if (work_count >= EDGE_RECORD_WRITE_BLOCK)
    {
        return false;
    }
    edge_work_buffer[work_count++] = value;
    return true;
}

static bool edge_record_emit_pending_long(void)
{
    while (long_pending && (work_count < EDGE_RECORD_WRITE_BLOCK))
    {
        if (long_first_pending)
        {
            uint32_t q = (long_units_remaining - 1UL) / 127UL;
            uint32_t r = long_units_remaining - q * 127UL;
            int8_t first = long_level ? (int8_t)r : (int8_t)(-(int16_t)r);
            if (!edge_record_emit_file_byte((uint8_t)first))
            {
                return false;
            }
            long_units_remaining = q;
            long_first_pending = false;
            if (long_units_remaining == 0UL)
            {
                long_pending = false;
            }
            continue;
        }

        if (!edge_record_emit_file_byte(0U))
        {
            return false;
        }
        long_units_remaining--;
        if (long_units_remaining == 0UL)
        {
            long_pending = false;
        }
    }
    return true;
}

/* Correct token parser that does not consume the marker before all bytes exist. */
static bool edge_record_consume_tokens_safe(void)
{
    while (work_count < EDGE_RECORD_WRITE_BLOCK)
    {
        uint8_t value;
        uint16_t available;

        if (long_pending)
        {
            if (!edge_record_emit_pending_long())
            {
                return false;
            }
            if (long_pending || (work_count >= EDGE_RECORD_WRITE_BLOCK))
            {
                return true;
            }
            continue;
        }

        available = edge_record_driver_available_bytes();
        if (available == 0U)
        {
            return true;
        }

        /* Peek by consuming normal byte only. Extended token is emitted atomically by ISR. */
        if (!edge_record_driver_pop_byte(&value))
        {
            return true;
        }

        if (value == EDGE_RECORD_EXTENDED_TOKEN)
        {
            uint8_t tail[5];
            if (available < EDGE_RECORD_EXTENDED_TOKEN_BYTES)
            {
                edge_record_set_error("TOKEN SPLIT");
                record_state = EDGE_RECORD_ENGINE_ERROR;
                return false;
            }
            for (uint8_t i = 0U; i < 5U; ++i)
            {
                if (!edge_record_driver_pop_byte(&tail[i]))
                {
                    edge_record_set_error("TOKEN SPLIT");
                    record_state = EDGE_RECORD_ENGINE_ERROR;
                    return false;
                }
            }
            long_level = tail[0] ? 1U : 0U;
            long_units_remaining = ((uint32_t)tail[1]) |
                                   ((uint32_t)tail[2] << 8) |
                                   ((uint32_t)tail[3] << 16) |
                                   ((uint32_t)tail[4] << 24);
            long_first_pending = true;
            long_pending = true;
            continue;
        }

        if (!edge_record_emit_file_byte(value))
        {
            return false;
        }
    }
    return true;
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
    work_count = 0U;
    long_pending = false;
    long_level = 0U;
    long_units_remaining = 0UL;
    long_first_pending = false;
}

bool edge_record_engine_preview_filename(const char *directory_path, file_format_t format)
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

    if (!edge_record_driver_prepare(unit_us) || !edge_record_make_paths(directory_path))
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

    if (!edge_record_driver_start())
    {
        edge_record_fail_close("EDGE START");
        return false;
    }

    record_state = EDGE_RECORD_ENGINE_RECORDING;
    return true;
}

bool edge_record_engine_pause(void)
{
    if ((record_state == EDGE_RECORD_ENGINE_RECORDING) && edge_record_driver_pause())
    {
        record_state = EDGE_RECORD_ENGINE_PAUSED;
        return true;
    }
    return false;
}

bool edge_record_engine_resume(void)
{
    if ((record_state == EDGE_RECORD_ENGINE_PAUSED) && edge_record_driver_resume())
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

        if (!edge_record_flush_work(false))
        {
            edge_record_fail_close(record_error_text);
            return;
        }
        if (work_count < EDGE_RECORD_WRITE_BLOCK && !edge_record_consume_tokens_safe())
        {
            edge_record_fail_close(record_error_text);
        }
        return;
    }

    if (record_state == EDGE_RECORD_ENGINE_FINALIZING)
    {
        /*
           Drain in the same order as active capture: write a completed (or
           final partial) work block first, then move more ISR tokens into it.
           The former code only consumed here. Once work_count reached 512,
           consume_tokens_safe() could no longer advance while remaining FIFO
           data prevented the close condition: an actual SAVING deadlock.
        */
        if (!edge_record_flush_work(true))
        {
            edge_record_fail_close(record_error_text);
            return;
        }
        if (work_count != 0U)
        {
            /* SD card is still busy; service will retry without blocking. */
            return;
        }

        if (!edge_record_consume_tokens_safe())
        {
            edge_record_fail_close(record_error_text);
            return;
        }

        if ((edge_record_driver_available_bytes() != 0U) || long_pending)
        {
            return;
        }

        /* consume_tokens_safe() may have produced the final block. */
        if (!edge_record_flush_work(true))
        {
            edge_record_fail_close(record_error_text);
            return;
        }
        if (work_count != 0U || sdcard_file_is_busy())
        {
            return;
        }

        if (!sdcard_file_sync())
        {
            edge_record_fail_close("EDGE CLOSE");
            return;
        }
        sdcard_file_close();
        record_state = EDGE_RECORD_ENGINE_FINISHED;
    }
}

edge_record_engine_state_t edge_record_engine_get_state(void) { return record_state; }
const char *edge_record_engine_get_filename(void) { return record_filename; }
const char *edge_record_engine_get_full_path(void) { return record_full_path; }
const char *edge_record_engine_get_error_text(void) { return record_error_text; }
uint8_t edge_record_engine_get_buffer_fill_percent(void)
{
    return edge_record_driver_fill_percent();
}

uint8_t edge_record_engine_get_buffer_headroom_percent(void)
{
    const uint32_t capacity = (uint32_t)(WAV_SAMPLE_STREAM_BUFFER_BYTES - 1U) +
                              (uint32_t)EDGE_RECORD_WRITE_BLOCK;
    uint32_t used = (uint32_t)edge_record_driver_available_bytes() +
                    (uint32_t)work_count;
    uint32_t free_bytes;

    if (used > capacity)
    {
        used = capacity;
    }
    free_bytes = capacity - used;
    return (uint8_t)((free_bytes * 100UL) / capacity);
}
