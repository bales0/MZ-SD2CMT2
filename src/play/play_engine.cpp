#include <Arduino.h>
#include <string.h>

#include "play_engine.h"
#include "edge_playback.h"
#include "../drivers/mzio.h"
#include "../drivers/wav_playback_driver.h"
#include "../streams/wav_sample_stream.h"

#define PLAY_ENGINE_PATH_MAX 160
#define PLAY_ENGINE_ERROR_TEXT_MAX 17
#define WAV_STREAM_REFILL_RESERVE \
    (WAV_SAMPLE_STREAM_CAPACITY_SAMPLES - WAV_SAMPLE_STREAM_REFILL_BLOCK)

static char prepared_full_path[PLAY_ENGINE_PATH_MAX];
static file_format_t prepared_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t prepared_play_mode = MENU_PLAY_MODE_NORMAL;
static bool prepared_invert_signal = false;
static play_engine_state_t engine_state = PLAY_ENGINE_STATE_STOPPED;
static char engine_error_text[PLAY_ENGINE_ERROR_TEXT_MAX] = "";

/* Generic source counters: WAV samples for WAV, file bytes for LEP/L16. */
static uint32_t source_total = 0UL;
static uint32_t source_played = 0UL;
static uint16_t wav_last_emitted_sequence = 0U;
/* True after EOF: SELECT starts the source again from byte/sample zero. */
static bool source_reprepare_required = false;

static void play_engine_clear_error(void)
{
    engine_error_text[0] = '\0';
}

static void play_engine_set_error(const char *text)
{
    if (text == NULL) text = "PLAY ERROR";
    strncpy(engine_error_text, text, sizeof(engine_error_text) - 1U);
    engine_error_text[sizeof(engine_error_text) - 1U] = '\0';
    engine_state = PLAY_ENGINE_STATE_ERROR;
    mz_sense_set(true);
}

static const char *play_engine_driver_error_text(wav_playback_driver_state_t state)
{
    switch (state)
    {
        case WAV_PLAYBACK_DRIVER_UNDERRUN: return "UNDERRUN";
        case WAV_PLAYBACK_DRIVER_BAD_RATE: return "BAD TIMER";
        case WAV_PLAYBACK_DRIVER_BAD_ARGUMENT: return "TIMER ARG";
        default: return "TIMER";
    }
}

static void play_engine_sync_wav_progress(void)
{
    uint16_t current_sequence;
    uint16_t delta;

    if (prepared_format != FILE_FORMAT_WAV) return;

    current_sequence = wav_playback_driver_get_emitted_sequence();
    delta = (uint16_t)(current_sequence - wav_last_emitted_sequence);
    source_played += (uint32_t)delta;
    wav_last_emitted_sequence = current_sequence;
    if (source_played > source_total) source_played = source_total;
}

static bool play_engine_prepare_wav(void)
{
    wav_sample_stream_config_t stream_config;
    const wav_reader_info_t *info;

    wav_playback_driver_init();
    stream_config.low_threshold = 100U;
    stream_config.high_threshold = 155U;
    stream_config.invert_signal = prepared_invert_signal;

    if (!wav_sample_stream_open(prepared_full_path, &stream_config))
    {
        play_engine_set_error(wav_reader_status_text(wav_sample_stream_last_status()));
        return false;
    }
    if (!wav_sample_stream_prefill() || (wav_sample_stream_available() == 0U))
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

    source_total = info->data_size;
    source_played = 0UL;
    wav_last_emitted_sequence = 0U;

    if (!wav_playback_driver_prepare(info->sample_rate))
    {
        wav_sample_stream_close();
        play_engine_set_error(play_engine_driver_error_text(wav_playback_driver_get_state()));
        return false;
    }
    return true;
}

static bool play_engine_prepare_edge(void)
{
    uint8_t unit_us = (prepared_format == FILE_FORMAT_L16) ? 16U : 50U;
    if (!edge_playback_prepare(prepared_full_path, unit_us, prepared_invert_signal))
    {
        play_engine_set_error(edge_playback_get_error_text());
        return false;
    }
    source_total = edge_playback_get_total_bytes();
    source_played = 0UL;
    return true;
}

static bool play_engine_prepare_saved_source(void)
{
    bool ok;

    /*
        EOF leaves the low-level driver stopped. Reopen and prefill the source
        before a later SELECT starts it again; never attempt to restart a
        stopped Timer3 driver with an exhausted FIFO.
    */
    play_engine_stop();
    play_engine_clear_error();

    if (prepared_format == FILE_FORMAT_WAV)
    {
        ok = play_engine_prepare_wav();
    }
    else if ((prepared_format == FILE_FORMAT_LEP) ||
             (prepared_format == FILE_FORMAT_L16))
    {
        ok = play_engine_prepare_edge();
    }
    else
    {
        play_engine_set_error("MZF NOT READY");
        return false;
    }

    if (ok)
    {
        source_reprepare_required = false;
        engine_state = PLAY_ENGINE_STATE_READY;
    }
    return ok;
}

void play_engine_init(void)
{
    prepared_full_path[0] = '\0';
    prepared_format = FILE_FORMAT_UNKNOWN;
    prepared_play_mode = MENU_PLAY_MODE_NORMAL;
    prepared_invert_signal = false;
    engine_state = PLAY_ENGINE_STATE_STOPPED;
    source_total = 0UL;
    source_played = 0UL;
    wav_last_emitted_sequence = 0U;
    source_reprepare_required = false;
    play_engine_clear_error();
    wav_playback_driver_init();
    edge_playback_init();
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

    strncpy(prepared_full_path, config->full_path, sizeof(prepared_full_path) - 1U);
    prepared_full_path[sizeof(prepared_full_path) - 1U] = '\0';
    prepared_format = config->format;
    prepared_play_mode = config->play_mode;
    prepared_invert_signal = config->invert_signal;

    return play_engine_prepare_saved_source();
}

bool play_engine_start(void)
{
    if (engine_state != PLAY_ENGINE_STATE_READY) return false;

    if (source_reprepare_required && !play_engine_prepare_saved_source())
    {
        return false;
    }

    if (prepared_format == FILE_FORMAT_WAV)
    {
        if (!wav_playback_driver_start())
        {
            play_engine_set_error(play_engine_driver_error_text(wav_playback_driver_get_state()));
            return false;
        }
        mz_sense_set(false);
    }
    else
    {
        if (!edge_playback_start())
        {
            play_engine_set_error(edge_playback_get_error_text());
            return false;
        }
    }

    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

bool play_engine_pause(void)
{
    if (engine_state != PLAY_ENGINE_STATE_RUNNING) return false;

    if (prepared_format == FILE_FORMAT_WAV)
    {
        play_engine_sync_wav_progress();
        if (!wav_playback_driver_pause())
        {
            play_engine_set_error(play_engine_driver_error_text(wav_playback_driver_get_state()));
            return false;
        }
    }
    else if (!edge_playback_pause())
    {
        play_engine_set_error(edge_playback_get_error_text());
        return false;
    }

    engine_state = PLAY_ENGINE_STATE_PAUSED;
    return true;
}

bool play_engine_resume(void)
{
    if (engine_state != PLAY_ENGINE_STATE_PAUSED) return false;

    if (prepared_format == FILE_FORMAT_WAV)
    {
        if (!wav_playback_driver_resume())
        {
            play_engine_set_error(play_engine_driver_error_text(wav_playback_driver_get_state()));
            return false;
        }
    }
    else if (!edge_playback_resume())
    {
        play_engine_set_error(edge_playback_get_error_text());
        return false;
    }

    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

void play_engine_stop(void)
{
    wav_playback_driver_stop();
    wav_sample_stream_close();
    edge_playback_stop();
    mz_sense_set(true);
    source_total = 0UL;
    source_played = 0UL;
    wav_last_emitted_sequence = 0U;
    source_reprepare_required = false;
    engine_state = PLAY_ENGINE_STATE_STOPPED;
}

void play_engine_service(void)
{
    if (engine_state != PLAY_ENGINE_STATE_RUNNING) return;

    if (prepared_format == FILE_FORMAT_WAV)
    {
        wav_playback_driver_state_t driver_state;
        if (!wav_sample_stream_source_finished() &&
            (wav_sample_stream_available() <= WAV_STREAM_REFILL_RESERVE))
        {
            if (!wav_sample_stream_refill_block())
            {
                wav_playback_driver_stop();
                play_engine_set_error(wav_reader_status_text(wav_sample_stream_last_status()));
                return;
            }
        }
        play_engine_sync_wav_progress();
        driver_state = wav_playback_driver_get_state();
        if (driver_state == WAV_PLAYBACK_DRIVER_FINISHED)
        {
            wav_playback_driver_stop();
            mz_sense_set(true);
            source_reprepare_required = true;
            engine_state = PLAY_ENGINE_STATE_READY;
        }
        else if ((driver_state == WAV_PLAYBACK_DRIVER_UNDERRUN) ||
                 (driver_state == WAV_PLAYBACK_DRIVER_BAD_RATE) ||
                 (driver_state == WAV_PLAYBACK_DRIVER_BAD_ARGUMENT))
        {
            play_engine_set_error(play_engine_driver_error_text(driver_state));
        }
        return;
    }

    edge_playback_service();
    source_played = edge_playback_get_consumed_bytes();
    source_total = edge_playback_get_total_bytes();
    switch (edge_playback_get_state())
    {
        case EDGE_PLAYBACK_RUNNING:
            break;
        case EDGE_PLAYBACK_FINISHED:
            source_reprepare_required = true;
            engine_state = PLAY_ENGINE_STATE_READY;
            break;
        case EDGE_PLAYBACK_UNDERRUN:
        case EDGE_PLAYBACK_IO_ERROR:
        case EDGE_PLAYBACK_BAD_FILE:
            play_engine_set_error(edge_playback_get_error_text());
            break;
        default:
            break;
    }
}

play_engine_state_t play_engine_get_state(void) { return engine_state; }
const char *play_engine_get_error_text(void) { return engine_error_text; }
uint8_t play_engine_get_output_pin(void) { return wav_playback_driver_get_read_pin(); }
uint32_t play_engine_get_played_samples(void) { return source_played; }
uint32_t play_engine_get_total_samples(void) { return source_total; }
uint8_t play_engine_get_buffer_fill_percent(void)
{
    uint32_t percent;
    if ((prepared_format == FILE_FORMAT_LEP) || (prepared_format == FILE_FORMAT_L16))
        return edge_playback_get_buffer_fill_percent();
    if (prepared_format != FILE_FORMAT_WAV) return 0U;
    percent = ((uint32_t)wav_sample_stream_available() * 100UL) /
              WAV_SAMPLE_STREAM_CAPACITY_SAMPLES;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}
uint16_t play_engine_get_jitter_ticks(void) { return wav_playback_driver_get_jitter_ticks(); }
