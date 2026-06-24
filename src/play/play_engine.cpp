#include <Arduino.h>
#include <string.h>

#include "play_engine.h"
#include "../drivers/wav_playback_driver.h"
#include "../streams/wav_sample_stream.h"

#define PLAY_ENGINE_PATH_MAX 160
#define PLAY_ENGINE_ERROR_TEXT_MAX 13

/*
    A refill consumes exactly one 512-byte SD/WAV block per service pass.
    The main loop invokes service three times per pass. Keeping the FIFO near
    full avoids long foreground monopolization and preserves >350 ms of
    reserve at 44.1 kHz.
*/
#define WAV_STREAM_REFILL_RESERVE \
    (WAV_SAMPLE_STREAM_CAPACITY_SAMPLES - WAV_SAMPLE_STREAM_REFILL_BLOCK)

static char prepared_full_path[PLAY_ENGINE_PATH_MAX];
static file_format_t prepared_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t prepared_play_mode = MENU_PLAY_MODE_NORMAL;
static bool prepared_invert_signal = false;
static play_engine_state_t engine_state = PLAY_ENGINE_STATE_STOPPED;
static char engine_error_text[PLAY_ENGINE_ERROR_TEXT_MAX] = "";

static uint32_t wav_total_samples = 0;
static uint32_t wav_played_samples = 0;
static uint16_t wav_last_emitted_sequence = 0;

static void play_engine_clear_error(void)
{
    engine_error_text[0] = '\0';
}

static void play_engine_set_error(const char *text)
{
    if (text == NULL)
    {
        text = "UNKNOWN";
    }

    strncpy(engine_error_text, text, sizeof(engine_error_text) - 1U);
    engine_error_text[sizeof(engine_error_text) - 1U] = '\0';
    engine_state = PLAY_ENGINE_STATE_ERROR;
}

static const char *play_engine_driver_error_text(wav_playback_driver_state_t state)
{
    switch (state)
    {
        case WAV_PLAYBACK_DRIVER_UNDERRUN:
            return "UNDERRUN";

        case WAV_PLAYBACK_DRIVER_BAD_RATE:
            return "BAD TIMER";

        case WAV_PLAYBACK_DRIVER_BAD_ARGUMENT:
            return "TIMER ARG";

        default:
            return "TIMER";
    }
}

static void play_engine_sync_wav_progress(void)
{
    uint16_t current_sequence;
    uint16_t delta;

    if (prepared_format != FILE_FORMAT_WAV)
    {
        return;
    }

    current_sequence = wav_playback_driver_get_emitted_sequence();
    delta = (uint16_t)(current_sequence - wav_last_emitted_sequence);

    wav_played_samples += (uint32_t)delta;
    wav_last_emitted_sequence = current_sequence;

    if (wav_played_samples > wav_total_samples)
    {
        wav_played_samples = wav_total_samples;
    }
}

static bool play_engine_prepare_wav(void)
{
    wav_sample_stream_config_t stream_config;
    const wav_reader_info_t *info;

    /*
        RECORD owns Timer1/D15, while WAV PLAY owns Timer3/OC3B on D2.
        Reinitializing the playback driver at every new WAV session explicitly
        restores Timer3, the OC3B compare mode and PE4 direction after a
        completed or failed record session.
    */
    wav_playback_driver_init();

    stream_config.low_threshold = 100;
    stream_config.high_threshold = 155;
    stream_config.invert_signal = prepared_invert_signal;

    if (!wav_sample_stream_open(prepared_full_path, &stream_config))
    {
        play_engine_set_error(
            wav_reader_status_text(wav_sample_stream_last_status())
        );
        return false;
    }

    /*
        Fill all 16383 packed sample positions before Timer1 starts. This is
        allowed to take time because READ remains idle and has no deadline yet.
    */
    if (!wav_sample_stream_prefill() ||
        (wav_sample_stream_available() == 0U))
    {
        wav_sample_stream_close();
        play_engine_set_error("EMPTY WAV");
        return false;
    }

    info = wav_sample_stream_info();

    if (info == NULL)
    {
        wav_sample_stream_close();
        play_engine_set_error("NO WAV INFO");
        return false;
    }

    wav_total_samples = info->data_size;
    wav_played_samples = 0;
    wav_last_emitted_sequence = 0;

    if (!wav_playback_driver_prepare(info->sample_rate))
    {
        wav_sample_stream_close();
        play_engine_set_error(
            play_engine_driver_error_text(wav_playback_driver_get_state())
        );
        return false;
    }

    return true;
}

void play_engine_init(void)
{
    prepared_full_path[0] = '\0';
    prepared_format = FILE_FORMAT_UNKNOWN;
    prepared_play_mode = MENU_PLAY_MODE_NORMAL;
    prepared_invert_signal = false;
    engine_state = PLAY_ENGINE_STATE_STOPPED;

    wav_total_samples = 0;
    wav_played_samples = 0;
    wav_last_emitted_sequence = 0;

    play_engine_clear_error();

    wav_playback_driver_init();
}

bool play_engine_prepare(const play_engine_config_t *config)
{
    play_engine_stop();
    play_engine_clear_error();

    if ((config == NULL) || (config->full_path == NULL) ||
        (config->format == FILE_FORMAT_UNKNOWN))
    {
        play_engine_set_error("BAD SESSION");
        return false;
    }

    strncpy(prepared_full_path, config->full_path,
            sizeof(prepared_full_path) - 1U);
    prepared_full_path[sizeof(prepared_full_path) - 1U] = '\0';

    prepared_format = config->format;
    prepared_play_mode = config->play_mode;
    prepared_invert_signal = config->invert_signal;

    if ((prepared_format == FILE_FORMAT_WAV) && !play_engine_prepare_wav())
    {
        return false;
    }

    /*
        Other supported tape formats remain in their existing stub state.
    */
    engine_state = PLAY_ENGINE_STATE_READY;
    return true;
}

bool play_engine_start(void)
{
    if (engine_state != PLAY_ENGINE_STATE_READY)
    {
        return false;
    }

    if ((prepared_format == FILE_FORMAT_WAV) &&
        !wav_playback_driver_start())
    {
        play_engine_set_error(
            play_engine_driver_error_text(wav_playback_driver_get_state())
        );
        return false;
    }

    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

bool play_engine_pause(void)
{
    if (engine_state != PLAY_ENGINE_STATE_RUNNING)
    {
        return false;
    }

    play_engine_sync_wav_progress();

    if ((prepared_format == FILE_FORMAT_WAV) &&
        !wav_playback_driver_pause())
    {
        play_engine_set_error(
            play_engine_driver_error_text(wav_playback_driver_get_state())
        );
        return false;
    }

    engine_state = PLAY_ENGINE_STATE_PAUSED;
    return true;
}

bool play_engine_resume(void)
{
    if (engine_state != PLAY_ENGINE_STATE_PAUSED)
    {
        return false;
    }

    if ((prepared_format == FILE_FORMAT_WAV) &&
        !wav_playback_driver_resume())
    {
        play_engine_set_error(
            play_engine_driver_error_text(wav_playback_driver_get_state())
        );
        return false;
    }

    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

void play_engine_stop(void)
{
    wav_playback_driver_stop();
    wav_sample_stream_close();

    wav_total_samples = 0;
    wav_played_samples = 0;
    wav_last_emitted_sequence = 0;
    engine_state = PLAY_ENGINE_STATE_STOPPED;
}

void play_engine_service(void)
{
    wav_playback_driver_state_t driver_state;

    if (engine_state != PLAY_ENGINE_STATE_RUNNING)
    {
        return;
    }

    if (prepared_format != FILE_FORMAT_WAV)
    {
        return;
    }

    /*
        Keep the producer ahead continuously, not only after half of the
        buffer has drained. One block per pass bounds foreground work.
    */
    if (!wav_sample_stream_source_finished() &&
        (wav_sample_stream_available() <= WAV_STREAM_REFILL_RESERVE))
    {
        if (!wav_sample_stream_refill_block())
        {
            wav_playback_driver_stop();
            play_engine_set_error(
                wav_reader_status_text(wav_sample_stream_last_status())
            );
            return;
        }
    }

    play_engine_sync_wav_progress();

    driver_state = wav_playback_driver_get_state();

    switch (driver_state)
    {
        case WAV_PLAYBACK_DRIVER_RUNNING:
            break;

        case WAV_PLAYBACK_DRIVER_FINISHED:
            /*
                Source EOF was reached exactly at a sample boundary. Re-open
                and prefill immediately so SELECT can replay the same file.
            */
            wav_playback_driver_stop();

            if (play_engine_prepare_wav())
            {
                engine_state = PLAY_ENGINE_STATE_READY;
            }
            break;

        case WAV_PLAYBACK_DRIVER_UNDERRUN:
        case WAV_PLAYBACK_DRIVER_BAD_RATE:
        case WAV_PLAYBACK_DRIVER_BAD_ARGUMENT:
            play_engine_set_error(
                play_engine_driver_error_text(driver_state)
            );
            break;

        case WAV_PLAYBACK_DRIVER_READY:
        case WAV_PLAYBACK_DRIVER_PAUSED:
        case WAV_PLAYBACK_DRIVER_STOPPED:
        default:
            break;
    }
}

play_engine_state_t play_engine_get_state(void)
{
    return engine_state;
}

const char *play_engine_get_error_text(void)
{
    return engine_error_text;
}

uint8_t play_engine_get_output_pin(void)
{
    return wav_playback_driver_get_read_pin();
}

uint32_t play_engine_get_played_samples(void)
{
    return wav_played_samples;
}

uint32_t play_engine_get_total_samples(void)
{
    return wav_total_samples;
}

uint8_t play_engine_get_buffer_fill_percent(void)
{
    uint32_t percent;

    if (prepared_format != FILE_FORMAT_WAV)
    {
        return 0U;
    }

    percent = ((uint32_t)wav_sample_stream_available() * 100UL) /
              WAV_SAMPLE_STREAM_CAPACITY_SAMPLES;

    if (percent > 100UL)
    {
        percent = 100UL;
    }

    return (uint8_t)percent;
}

uint16_t play_engine_get_jitter_ticks(void)
{
    return wav_playback_driver_get_jitter_ticks();
}
