#include "wav_record_engine.h"

#include <stdio.h>
#include <string.h>

#include "../drivers/sdcard.h"
#include "../drivers/wav_record_driver.h"
#include "../streams/record_sample_stream.h"
#include "../streams/wav_sample_stream.h"
#include "../drivers/flash_text.h"

#define WAV_RECORD_PATH_MAX 160U
#define WAV_RECORD_FILENAME_MAX 16U

/* 64 packed bytes expand exactly to one 512-byte PCM sector. */
#define WAV_RECORD_RAW_BLOCK_BYTES 64U
#define WAV_RECORD_PCM_BLOCK_BYTES 512U

/* RIFF + fmt + JUNK + data header. Data begins aligned at byte 512. */
#define WAV_RECORD_HEADER_BYTES 512U
#define WAV_RECORD_JUNK_PAYLOAD_BYTES 460U

/*
    2 MiB = about 47.6 s at 44.1 kHz, 8-bit mono. Reservation happens before
    Timer1 starts, so the filesystem never needs to find a cluster mid-record.
*/
#define WAV_RECORD_PREALLOCATE_PCM_BYTES (2UL * 1024UL * 1024UL)

static wav_record_engine_state_t record_state = WAV_RECORD_ENGINE_STOPPED;

static char record_full_path[WAV_RECORD_PATH_MAX];
static char record_filename[WAV_RECORD_FILENAME_MAX];
static char record_error_text[17];

static uint32_t record_sample_rate = 0UL;
static uint32_t data_bytes_written = 0UL;
static uint8_t final_tail_byte = 0U;
static uint8_t final_tail_bits = 0U;
static bool final_tail_taken = false;

#define wav_record_work_block wav_sample_stream_get_shared_work_buffer()

static void wav_record_set_error(const char *text)
{
    if (text == NULL)
    {
        record_error_text[0] = '\0';
        return;
    }

    strncpy(record_error_text, text, sizeof(record_error_text) - 1U);
    record_error_text[sizeof(record_error_text) - 1U] = '\0';
}

static void wav_record_set_error_P(PGM_P text)
{
    flash_text_copy(record_error_text, sizeof(record_error_text), text);
}

static void wav_record_put_le16(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)(value & 0xFFU);
    target[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void wav_record_put_le32(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)(value & 0xFFUL);
    target[1] = (uint8_t)((value >> 8) & 0xFFUL);
    target[2] = (uint8_t)((value >> 16) & 0xFFUL);
    target[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

static bool wav_record_make_paths(const char *directory_path)
{
    uint16_t sequence;
    int length;

    if (!sdcard_next_record_sequence(directory_path, &sequence))
    {
        wav_record_set_error(sdcard_last_error());
        return false;
    }

    length = flash_text_snprintf(record_full_path, sizeof(record_full_path),
                                 PSTR("%s/REC%04u.WAV"), directory_path,
                                 (unsigned int)sequence);

    if ((length <= 0) || ((size_t)length >= sizeof(record_full_path)))
    {
        wav_record_set_error_P(PSTR("PATH TOO LONG"));
        return false;
    }

    flash_text_snprintf(record_filename, sizeof(record_filename),
                        PSTR("REC%04u.WAV"), (unsigned int)sequence);
    return true;
}

static void wav_record_stop_close_error(const char *text)
{
    /* Existing engine errors already live in this buffer; do not copy an
       overlapping C string onto itself. */
    if ((text != NULL) && (text != record_error_text))
    {
        wav_record_set_error(text);
    }

    wav_record_driver_stop();
    sdcard_file_close();
    record_state = WAV_RECORD_ENGINE_ERROR;
}

static void wav_record_stop_close_error_P(PGM_P text)
{
    if (text != NULL)
    {
        wav_record_set_error_P(text);
    }

    wav_record_driver_stop();
    sdcard_file_close();
    record_state = WAV_RECORD_ENGINE_ERROR;
}

static bool wav_record_write_header(uint32_t data_bytes)
{
    uint32_t padded_data_bytes = data_bytes + (data_bytes & 1UL);
    uint32_t file_bytes = WAV_RECORD_HEADER_BYTES + padded_data_bytes;

    if (file_bytes < WAV_RECORD_HEADER_BYTES)
    {
        wav_record_set_error_P(PSTR("WAV TOO BIG"));
        return false;
    }

    memset(wav_record_work_block, 0, WAV_RECORD_HEADER_BYTES);

    wav_record_work_block[0U] = 'R';
    wav_record_work_block[1U] = 'I';
    wav_record_work_block[2U] = 'F';
    wav_record_work_block[3U] = 'F';
    wav_record_put_le32(wav_record_work_block + 4U, file_bytes - 8UL);
    wav_record_work_block[8U] = 'W';
    wav_record_work_block[9U] = 'A';
    wav_record_work_block[10U] = 'V';
    wav_record_work_block[11U] = 'E';

    wav_record_work_block[12U] = 'f';
    wav_record_work_block[13U] = 'm';
    wav_record_work_block[14U] = 't';
    wav_record_work_block[15U] = ' ';
    wav_record_put_le32(wav_record_work_block + 16U, 16UL);
    wav_record_put_le16(wav_record_work_block + 20U, 1U);
    wav_record_put_le16(wav_record_work_block + 22U, 1U);
    wav_record_put_le32(wav_record_work_block + 24U, record_sample_rate);
    wav_record_put_le32(wav_record_work_block + 28U, record_sample_rate);
    wav_record_put_le16(wav_record_work_block + 32U, 1U);
    wav_record_put_le16(wav_record_work_block + 34U, 8U);

    /* JUNK pads the standard data header so audio starts at sector 512. */
    wav_record_work_block[36U] = 'J';
    wav_record_work_block[37U] = 'U';
    wav_record_work_block[38U] = 'N';
    wav_record_work_block[39U] = 'K';
    wav_record_put_le32(wav_record_work_block + 40U,
                        WAV_RECORD_JUNK_PAYLOAD_BYTES);
    wav_record_work_block[504U] = 'd';
    wav_record_work_block[505U] = 'a';
    wav_record_work_block[506U] = 't';
    wav_record_work_block[507U] = 'a';
    wav_record_put_le32(wav_record_work_block + 508U, data_bytes);

    if (!sdcard_file_seek(0UL) ||
        (sdcard_file_write(wav_record_work_block, WAV_RECORD_HEADER_BYTES) !=
         (int16_t)WAV_RECORD_HEADER_BYTES))
    {
        wav_record_set_error_P(PSTR("WAV HEADER ERR"));
        return false;
    }

    return true;
}

/*
    work[0..raw_count-1] initially contains packed samples. Expand backwards
    in the same 512-byte block, then append standard PCM bytes to the WAV.
*/
static bool wav_record_expand_and_write(uint16_t raw_count)
{
    uint16_t pcm_count;

    if ((raw_count == 0U) || (raw_count > WAV_RECORD_RAW_BLOCK_BYTES))
    {
        wav_record_set_error_P(PSTR("RAW BLOCK ERR"));
        return false;
    }

    for (uint16_t byte_index = raw_count; byte_index != 0U; byte_index--)
    {
        uint16_t input_index = (uint16_t)(byte_index - 1U);
        uint8_t packed = wav_record_work_block[input_index];
        uint16_t output_base = (uint16_t)(input_index * 8U);

        for (uint8_t bit_index = 8U; bit_index != 0U; bit_index--)
        {
            uint8_t bit = (uint8_t)(bit_index - 1U);
            wav_record_work_block[output_base + bit] =
                (packed & (uint8_t)(1U << bit)) ? 235U : 20U;
        }
    }

    pcm_count = (uint16_t)(raw_count * 8U);

    if (sdcard_file_write(wav_record_work_block, pcm_count) !=
        (int16_t)pcm_count)
    {
        wav_record_set_error_P(PSTR("WAV WRITE ERR"));
        return false;
    }

    data_bytes_written += (uint32_t)pcm_count;
    return true;
}

static bool wav_record_drain_one_block_if_ready(bool allow_partial)
{
    uint16_t available = record_sample_stream_available_bytes();
    uint16_t request;
    uint16_t got;

    if (available == 0U)
    {
        return true;
    }
    if (!allow_partial && (available < WAV_RECORD_RAW_BLOCK_BYTES))
    {
        return true;
    }
    if (sdcard_file_is_busy())
    {
        return true;
    }

    request = available;
    if (request > WAV_RECORD_RAW_BLOCK_BYTES)
    {
        request = WAV_RECORD_RAW_BLOCK_BYTES;
    }

    got = record_sample_stream_pop_bytes(wav_record_work_block, request);
    if ((got == 0U) || !wav_record_expand_and_write(got))
    {
        return false;
    }
    return true;
}

static bool wav_record_write_final_tail(void)
{
    if (final_tail_taken)
    {
        return true;
    }

    final_tail_taken = true;
    final_tail_byte = 0U;
    final_tail_bits = 0U;
    (void)wav_record_driver_take_tail(&final_tail_byte, &final_tail_bits);

    if (final_tail_bits == 0U)
    {
        return true;
    }

    for (uint8_t bit = 0U; bit < final_tail_bits; bit++)
    {
        wav_record_work_block[bit] =
            (final_tail_byte & (uint8_t)(1U << bit)) ? 235U : 20U;
    }

    if (sdcard_file_write(wav_record_work_block, final_tail_bits) !=
        (int16_t)final_tail_bits)
    {
        wav_record_set_error_P(PSTR("TAIL WRITE ERR"));
        return false;
    }

    data_bytes_written += (uint32_t)final_tail_bits;
    return true;
}

static bool wav_record_finalize_file(void)
{
    uint32_t final_file_bytes = WAV_RECORD_HEADER_BYTES + data_bytes_written;

    if ((data_bytes_written & 1UL) != 0UL)
    {
        wav_record_work_block[0] = 0U;

        if (sdcard_file_write(wav_record_work_block, 1U) != 1)
        {
            wav_record_set_error_P(PSTR("WAV PAD ERR"));
            return false;
        }

        final_file_bytes++;
    }

    if (!wav_record_write_header(data_bytes_written) ||
        !sdcard_file_truncate(final_file_bytes) ||
        !sdcard_file_sync())
    {
        if (record_error_text[0] == '\0')
        {
            wav_record_set_error_P(PSTR("WAV CLOSE ERR"));
        }
        return false;
    }

    sdcard_file_close();
    record_state = WAV_RECORD_ENGINE_FINISHED;
    return true;
}

void wav_record_engine_init(void)
{
    wav_record_driver_init();

    record_state = WAV_RECORD_ENGINE_STOPPED;
    record_full_path[0] = '\0';
    record_filename[0] = '\0';
    record_error_text[0] = '\0';
    record_sample_rate = 0UL;
    data_bytes_written = 0UL;
    final_tail_byte = 0U;
    final_tail_bits = 0U;
    final_tail_taken = false;
}

bool wav_record_engine_preview_filename(const char *directory_path)
{
    /* This only scans for the next free name. It does not create/open a file. */
    return wav_record_make_paths(directory_path);
}

bool wav_record_engine_start(const char *directory_path,
                             uint32_t sample_rate)
{
    wav_record_engine_init();

    if (!sdcard_is_mounted())
    {
        wav_record_set_error_P(PSTR("SD CARD ERROR"));
        record_state = WAV_RECORD_ENGINE_ERROR;
        return false;
    }

    /* PLAY and RECORD cannot overlap. Reset old PLAY stream ownership. */
    wav_sample_stream_close();

    if (!wav_record_driver_prepare(sample_rate))
    {
        wav_record_set_error_P(PSTR("BAD REC RATE"));
        record_state = WAV_RECORD_ENGINE_ERROR;
        return false;
    }

    if (!wav_record_make_paths(directory_path))
    {
        record_state = WAV_RECORD_ENGINE_ERROR;
        return false;
    }

    record_sample_rate = sample_rate;

    if (!sdcard_file_open_write(record_full_path))
    {
        wav_record_set_error_P(PSTR("WAV CREATE ERR"));
        record_state = WAV_RECORD_ENGINE_ERROR;
        return false;
    }

    /*
        Contiguous allocation must complete before Timer1 starts. Starting a
        realtime direct-PCM record without it would allow a FAT allocation
        pause to overflow the 371 ms packed FIFO.
    */
    if (!sdcard_file_preallocate(WAV_RECORD_PREALLOCATE_PCM_BYTES))
    {
        wav_record_stop_close_error_P(PSTR("PREALLOC FAIL"));
        return false;
    }

    if (!sdcard_file_seek(0UL) ||
        !wav_record_write_header(0UL) ||
        !sdcard_file_seek(WAV_RECORD_HEADER_BYTES) ||
        !sdcard_file_sync())
    {
        wav_record_stop_close_error(record_error_text);
        return false;
    }

    if (!wav_record_driver_start())
    {
        wav_record_stop_close_error_P(PSTR("REC TIMER ERR"));
        return false;
    }

    record_state = WAV_RECORD_ENGINE_RECORDING;
    return true;
}

bool wav_record_engine_pause(void)
{
    if ((record_state == WAV_RECORD_ENGINE_RECORDING) && wav_record_driver_pause())
    {
        record_state = WAV_RECORD_ENGINE_PAUSED;
        return true;
    }
    return false;
}

bool wav_record_engine_resume(void)
{
    if ((record_state == WAV_RECORD_ENGINE_PAUSED) && wav_record_driver_resume())
    {
        record_state = WAV_RECORD_ENGINE_RECORDING;
        return true;
    }
    return false;
}

void wav_record_engine_cancel(void)
{
    wav_record_driver_stop();
    sdcard_file_close();
    if (record_full_path[0] != '\0')
    {
        (void)sdcard_file_remove(record_full_path);
    }
    record_state = WAV_RECORD_ENGINE_STOPPED;
    data_bytes_written = 0UL;
    final_tail_taken = false;
}

void wav_record_engine_request_stop(void)
{
    if ((record_state != WAV_RECORD_ENGINE_RECORDING) &&
        (record_state != WAV_RECORD_ENGINE_PAUSED))
    {
        return;
    }

    wav_record_driver_stop();
    record_state = WAV_RECORD_ENGINE_FINALIZING;
}

void wav_record_engine_service(void)
{
    if ((record_state == WAV_RECORD_ENGINE_RECORDING) ||
        (record_state == WAV_RECORD_ENGINE_PAUSED))
    {
        if (wav_record_driver_get_state() == WAV_RECORD_DRIVER_OVERRUN)
        {
            wav_record_stop_close_error_P(PSTR("REC OVERFLOW"));
            return;
        }

        if (!wav_record_drain_one_block_if_ready(false))
        {
            wav_record_stop_close_error(record_error_text);
        }

        return;
    }

    if (record_state == WAV_RECORD_ENGINE_FINALIZING)
    {
        /* Never drain an arbitrary number of blocks or wait for SD busy here.
           One foreground service pass performs at most one data write. */
        if (record_sample_stream_available_bytes() != 0U)
        {
            if (!wav_record_drain_one_block_if_ready(true))
            {
                wav_record_stop_close_error(record_error_text);
            }
            return;
        }

        if (!final_tail_taken)
        {
            if (sdcard_file_is_busy())
            {
                return;
            }
            if (!wav_record_write_final_tail())
            {
                wav_record_stop_close_error(record_error_text);
            }
            return;
        }

        if (sdcard_file_is_busy())
        {
            return;
        }
        if (!wav_record_finalize_file())
        {
            wav_record_stop_close_error(record_error_text);
        }
    }
}

wav_record_engine_state_t wav_record_engine_get_state(void)
{
    return record_state;
}

const char *wav_record_engine_get_filename(void)
{
    return record_filename;
}

const char *wav_record_engine_get_full_path(void)
{
    return record_full_path;
}

const char *wav_record_engine_get_error_text(void)
{
    return record_error_text;
}

uint32_t wav_record_engine_get_sample_rate(void)
{
    return record_sample_rate;
}

uint32_t wav_record_engine_get_captured_samples(void)
{
    uint32_t samples = data_bytes_written;

    if ((record_state == WAV_RECORD_ENGINE_RECORDING) ||
        (record_state == WAV_RECORD_ENGINE_PAUSED))
    {
        samples += (uint32_t)record_sample_stream_available_bytes() * 8UL;
        samples += (uint32_t)wav_record_driver_get_pending_sample_count();
    }

    return samples;
}

uint8_t wav_record_engine_get_buffer_fill_percent(void)
{
    return record_sample_stream_fill_percent();
}

uint8_t wav_record_engine_get_buffer_headroom_percent(void)
{
    return (uint8_t)(100U - record_sample_stream_fill_percent());
}

uint8_t wav_record_engine_get_write_pin(void)
{
    return wav_record_driver_get_write_pin();
}
