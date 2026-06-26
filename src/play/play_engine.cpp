#include <Arduino.h>
#include <string.h>

#include "play_engine.h"
#include "edge_playback.h"
#include "mzf_playback.h"
#include "../drivers/mzio.h"
#include "../drivers/flash_text.h"
#include "../drivers/wav_playback_driver.h"
#include "../streams/wav_sample_stream.h"

#define PLAY_ENGINE_PATH_MAX 160
#define PLAY_ENGINE_ERROR_TEXT_MAX 17
#define WAV_STREAM_REFILL_RESERVE \
    (WAV_SAMPLE_STREAM_CAPACITY_SAMPLES - WAV_SAMPLE_STREAM_REFILL_BLOCK)

static char prepared_full_path[PLAY_ENGINE_PATH_MAX];
static file_format_t prepared_format = FILE_FORMAT_UNKNOWN;
static bool prepared_invert_signal = false;
static play_engine_state_t engine_state = PLAY_ENGINE_STATE_STOPPED;
static char engine_error_text[PLAY_ENGINE_ERROR_TEXT_MAX];

/* Active-time clock: it starts with output and stops for MANUAL/MOTOR pauses. */
static uint32_t elapsed_ms = 0UL;
static uint32_t active_started_ms = 0UL;
static uint32_t total_duration_ms = 0UL;
static bool active_clock_running = false;

/* True after EOF: SELECT starts the source again from the beginning. */
static bool source_reprepare_required = false;

static uint32_t play_engine_add_saturating(uint32_t left, uint32_t right)
{
    if ((0xFFFFFFFFUL - left) < right) return 0xFFFFFFFFUL;
    return left + right;
}

static uint32_t play_engine_duration_from_samples(uint32_t samples,
                                                   uint32_t sample_rate)
{
    uint32_t whole_seconds;
    uint32_t remainder_samples;

    if (sample_rate == 0UL) return 0UL;
    whole_seconds = samples / sample_rate;
    remainder_samples = samples % sample_rate;

    if (whole_seconds > 4294967UL) return 0xFFFFFFFFUL;
    return whole_seconds * 1000UL +
           (uint32_t)(((remainder_samples * 1000UL) + (sample_rate / 2UL)) /
                      sample_rate);
}

static void play_engine_clock_reset(void)
{
    elapsed_ms = 0UL;
    active_started_ms = 0UL;
    active_clock_running = false;
}

static void play_engine_clock_start(void)
{
    if (!active_clock_running)
    {
        active_started_ms = millis();
        active_clock_running = true;
    }
}

static void play_engine_clock_pause(void)
{
    if (active_clock_running)
    {
        elapsed_ms = play_engine_add_saturating(elapsed_ms,
                                                (uint32_t)(millis() - active_started_ms));
        active_clock_running = false;
    }
}

static uint32_t play_engine_clock_elapsed_ms(void)
{
    if (!active_clock_running) return elapsed_ms;
    return play_engine_add_saturating(elapsed_ms,
                                      (uint32_t)(millis() - active_started_ms));
}

static void play_engine_clock_finish(void)
{
    play_engine_clock_pause();
    /* WAV/MZF/MZT/M12 have an exact nominal endpoint. LEP/L16 uses percent. */
    if (total_duration_ms != 0UL) elapsed_ms = total_duration_ms;
}

static void play_engine_clear_error(void)
{
    engine_error_text[0] = '\0';
}

static void play_engine_set_error(const char *text)
{
    if (text == NULL)
    {
        flash_text_copy(engine_error_text, sizeof(engine_error_text), PSTR("PLAY ERROR"));
    }
    else
    {
        strncpy(engine_error_text, text, sizeof(engine_error_text) - 1U);
        engine_error_text[sizeof(engine_error_text) - 1U] = '\0';
    }
    play_engine_clock_pause();
    engine_state = PLAY_ENGINE_STATE_ERROR;
    mz_sense_set(true);
}

static void play_engine_set_error_P(PGM_P text)
{
    flash_text_copy(engine_error_text, sizeof(engine_error_text), text);
    play_engine_clock_pause();
    engine_state = PLAY_ENGINE_STATE_ERROR;
    mz_sense_set(true);
}

static PGM_P play_engine_driver_error_text_P(wav_playback_driver_state_t state)
{
    switch (state)
    {
        case WAV_PLAYBACK_DRIVER_UNDERRUN: return PSTR("UNDERRUN");
        case WAV_PLAYBACK_DRIVER_BAD_RATE: return PSTR("BAD TIMER");
        case WAV_PLAYBACK_DRIVER_BAD_ARGUMENT: return PSTR("TIMER ARG");
        default: return PSTR("TIMER");
    }
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
        play_engine_set_error_P(wav_reader_status_text_P(wav_sample_stream_last_status()));
        return false;
    }
    if (!wav_sample_stream_prefill() || (wav_sample_stream_available() == 0U))
    {
        wav_sample_stream_close();
        play_engine_set_error_P(PSTR("EMPTY WAV"));
        return false;
    }
    info = wav_sample_stream_info();
    if (info == NULL)
    {
        wav_sample_stream_close();
        play_engine_set_error_P(PSTR("NO WAV INFO"));
        return false;
    }

    total_duration_ms = play_engine_duration_from_samples(info->data_size,
                                                           info->sample_rate);

    if (!wav_playback_driver_prepare(info->sample_rate))
    {
        wav_sample_stream_close();
        play_engine_set_error_P(play_engine_driver_error_text_P(wav_playback_driver_get_state()));
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
    /* LEP/L16 use immediate byte progress; do not scan the file for duration. */
    total_duration_ms = 0UL;
    return true;
}

static bool play_engine_prepare_mzf(void)
{
    /* MZF/MZT/M12 use their fixed native polarity; INVERT SIG. applies
       only to sampled WAV and LEP/L16 transports. */
    if (!mzf_playback_prepare(prepared_full_path, prepared_format))
    {
        play_engine_set_error(mzf_playback_get_error_text());
        return false;
    }
    total_duration_ms = mzf_playback_get_total_duration_ms();
    return true;
}

static bool play_engine_prepare_saved_source(void)
{
    bool ok;

    /* EOF leaves the low-level driver stopped. Reopen and prefill before a
       later SELECT starts it again; never restart an exhausted FIFO. */
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
    else if (file_format_is_sharp_tape(prepared_format))
    {
        ok = play_engine_prepare_mzf();
    }
    else
    {
        play_engine_set_error_P(PSTR("BAD FORMAT"));
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
    prepared_invert_signal = false;
    engine_state = PLAY_ENGINE_STATE_STOPPED;
    total_duration_ms = 0UL;
    play_engine_clock_reset();
    source_reprepare_required = false;
    play_engine_clear_error();
    wav_playback_driver_init();
    edge_playback_init();
    mzf_playback_init();
}

bool play_engine_prepare(const play_engine_config_t *config)
{
    play_engine_stop();
    play_engine_clear_error();

    if ((config == NULL) || (config->full_path == NULL) ||
        (config->format == FILE_FORMAT_UNKNOWN))
    {
        play_engine_set_error_P(PSTR("BAD SESSION"));
        return false;
    }

    strncpy(prepared_full_path, config->full_path, sizeof(prepared_full_path) - 1U);
    prepared_full_path[sizeof(prepared_full_path) - 1U] = '\0';
    prepared_format = config->format;
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
            play_engine_set_error_P(play_engine_driver_error_text_P(wav_playback_driver_get_state()));
            return false;
        }
        mz_sense_set(false);
    }
    else if (file_format_is_sharp_tape(prepared_format))
    {
        if (!mzf_playback_start())
        {
            play_engine_set_error(mzf_playback_get_error_text());
            return false;
        }
    }
    else
    {
        if (!edge_playback_start())
        {
            play_engine_set_error(edge_playback_get_error_text());
            return false;
        }
    }

    play_engine_clock_start();
    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

bool play_engine_pause(void)
{
    if (engine_state != PLAY_ENGINE_STATE_RUNNING) return false;

    if (prepared_format == FILE_FORMAT_WAV)
    {
        if (!wav_playback_driver_pause())
        {
            play_engine_set_error_P(play_engine_driver_error_text_P(wav_playback_driver_get_state()));
            return false;
        }
    }
    else if (file_format_is_sharp_tape(prepared_format))
    {
        if (!mzf_playback_pause())
        {
            play_engine_set_error(mzf_playback_get_error_text());
            return false;
        }
    }
    else if (!edge_playback_pause())
    {
        play_engine_set_error(edge_playback_get_error_text());
        return false;
    }

    play_engine_clock_pause();
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
            play_engine_set_error_P(play_engine_driver_error_text_P(wav_playback_driver_get_state()));
            return false;
        }
    }
    else if (file_format_is_sharp_tape(prepared_format))
    {
        if (!mzf_playback_resume())
        {
            play_engine_set_error(mzf_playback_get_error_text());
            return false;
        }
    }
    else if (!edge_playback_resume())
    {
        play_engine_set_error(edge_playback_get_error_text());
        return false;
    }

    play_engine_clock_start();
    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

void play_engine_stop(void)
{
    wav_playback_driver_stop();
    wav_sample_stream_close();
    edge_playback_stop();
    mzf_playback_stop();
    mz_sense_set(true);
    total_duration_ms = 0UL;
    play_engine_clock_reset();
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
                play_engine_set_error_P(wav_reader_status_text_P(wav_sample_stream_last_status()));
                return;
            }
        }
        driver_state = wav_playback_driver_get_state();
        if (driver_state == WAV_PLAYBACK_DRIVER_FINISHED)
        {
            wav_playback_driver_stop();
            mz_sense_set(true);
            play_engine_clock_finish();
            source_reprepare_required = true;
            engine_state = PLAY_ENGINE_STATE_READY;
        }
        else if ((driver_state == WAV_PLAYBACK_DRIVER_UNDERRUN) ||
                 (driver_state == WAV_PLAYBACK_DRIVER_BAD_RATE) ||
                 (driver_state == WAV_PLAYBACK_DRIVER_BAD_ARGUMENT))
        {
            play_engine_set_error_P(play_engine_driver_error_text_P(driver_state));
        }
        return;
    }

    if (file_format_is_sharp_tape(prepared_format))
    {
        mzf_playback_service();
        switch (mzf_playback_get_state())
        {
            case MZF_PLAYBACK_RUNNING:
            case MZF_PLAYBACK_PAUSED:
                break;
            case MZF_PLAYBACK_FINISHED:
                play_engine_clock_finish();
                source_reprepare_required = true;
                engine_state = PLAY_ENGINE_STATE_READY;
                break;
            case MZF_PLAYBACK_UNDERRUN:
            case MZF_PLAYBACK_IO_ERROR:
            case MZF_PLAYBACK_BAD_FILE:
                play_engine_set_error(mzf_playback_get_error_text());
                break;
            default:
                break;
        }
        return;
    }

    edge_playback_service();
    switch (edge_playback_get_state())
    {
        case EDGE_PLAYBACK_RUNNING:
            break;
        case EDGE_PLAYBACK_FINISHED:
            play_engine_clock_finish();
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
uint32_t play_engine_get_elapsed_ms(void) { return play_engine_clock_elapsed_ms(); }
uint32_t play_engine_get_total_duration_ms(void) { return total_duration_ms; }
uint8_t play_engine_get_progress_percent(void)
{
    if ((prepared_format == FILE_FORMAT_LEP) || (prepared_format == FILE_FORMAT_L16))
    {
        return edge_playback_get_progress_percent();
    }
    return 0U;
}
uint8_t play_engine_get_buffer_fill_percent(void)
{
    uint32_t percent;
    if ((prepared_format == FILE_FORMAT_LEP) || (prepared_format == FILE_FORMAT_L16))
        return edge_playback_get_buffer_fill_percent();
    if (file_format_is_sharp_tape(prepared_format))
        return mzf_playback_get_buffer_fill_percent();
    if (prepared_format != FILE_FORMAT_WAV) return 0U;
    percent = ((uint32_t)wav_sample_stream_available() * 100UL) /
              WAV_SAMPLE_STREAM_CAPACITY_SAMPLES;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}
uint16_t play_engine_get_jitter_ticks(void) { return wav_playback_driver_get_jitter_ticks(); }
