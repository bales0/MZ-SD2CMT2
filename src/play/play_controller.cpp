#include <Arduino.h>
#include <string.h>

#include "play_controller.h"
#include "play_engine.h"
#include "../drivers/wav_playback_driver.h"

static char session_filename[PLAY_CONTROLLER_NAME_MAX];
static char session_full_path[PLAY_CONTROLLER_PATH_MAX];
static file_format_t session_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t session_play_mode = MENU_PLAY_MODE_NORMAL;
static bool session_invert_signal = false;
static play_controller_state_t session_state = PLAY_CONTROLLER_STATE_READY;

static bool play_controller_can_start(void)
{
    return session_format != FILE_FORMAT_UNKNOWN;
}

static uint8_t play_controller_percent(uint32_t played, uint32_t total)
{
    uint32_t percent;

    if (total == 0UL)
    {
        return 0;
    }

    /*
        Avoid played*100 overflow for large WAV files while preserving a
        0..100 integer display value.
    */
    if (total >= 100UL)
    {
        percent = played / (total / 100UL);
    }
    else
    {
        percent = (played * 100UL) / total;
    }

    if (percent > 100UL)
    {
        percent = 100UL;
    }

    return (uint8_t)percent;
}

void play_controller_init(void)
{
    session_filename[0] = '\0';
    session_full_path[0] = '\0';
    session_format = FILE_FORMAT_UNKNOWN;
    session_play_mode = MENU_PLAY_MODE_NORMAL;
    session_invert_signal = false;
    session_state = PLAY_CONTROLLER_STATE_READY;

    play_engine_init();
}

void play_controller_start_session(const char *filename,
                                   const char *full_path,
                                   menu_play_mode_t play_mode,
                                   bool invert_signal)
{
    if (filename == NULL)
    {
        session_filename[0] = '\0';
    }
    else
    {
        strncpy(session_filename, filename, sizeof(session_filename) - 1U);
        session_filename[sizeof(session_filename) - 1U] = '\0';
    }

    if (full_path == NULL)
    {
        session_full_path[0] = '\0';
    }
    else
    {
        strncpy(session_full_path, full_path, sizeof(session_full_path) - 1U);
        session_full_path[sizeof(session_full_path) - 1U] = '\0';
    }

    session_format = file_format_detect_from_name(session_filename);
    session_play_mode = play_mode;
    session_invert_signal = invert_signal;
    session_state = PLAY_CONTROLLER_STATE_READY;

    if (session_format != FILE_FORMAT_UNKNOWN)
    {
        play_engine_config_t config;

        config.full_path = session_full_path;
        config.format = session_format;
        config.play_mode = session_play_mode;
        config.invert_signal = session_invert_signal;

        if (!play_engine_prepare(&config))
        {
            session_state = PLAY_CONTROLLER_STATE_ERROR;
        }
    }
    else
    {
        play_engine_stop();
    }
}

void play_controller_toggle_play_pause(void)
{
    if (!play_controller_can_start())
    {
        return;
    }

    switch (session_state)
    {
        case PLAY_CONTROLLER_STATE_READY:
            if (play_engine_start())
            {
                session_state = PLAY_CONTROLLER_STATE_PLAYING;
            }
            else
            {
                session_state = PLAY_CONTROLLER_STATE_ERROR;
            }
            break;

        case PLAY_CONTROLLER_STATE_PLAYING:
            if (play_engine_pause())
            {
                session_state = PLAY_CONTROLLER_STATE_PAUSED;
            }
            else
            {
                session_state = PLAY_CONTROLLER_STATE_ERROR;
            }
            break;

        case PLAY_CONTROLLER_STATE_PAUSED:
            if (play_engine_resume())
            {
                session_state = PLAY_CONTROLLER_STATE_PLAYING;
            }
            else
            {
                session_state = PLAY_CONTROLLER_STATE_ERROR;
            }
            break;

        case PLAY_CONTROLLER_STATE_ERROR:
        default:
            session_state = PLAY_CONTROLLER_STATE_READY;
            play_engine_stop();
            break;
    }
}

void play_controller_stop(void)
{
    play_engine_stop();
    session_state = PLAY_CONTROLLER_STATE_READY;
}

void play_controller_service(void)
{
    play_engine_state_t engine_state;

    play_engine_service();
    engine_state = play_engine_get_state();

    switch (engine_state)
    {
        case PLAY_ENGINE_STATE_RUNNING:
            session_state = PLAY_CONTROLLER_STATE_PLAYING;
            break;

        case PLAY_ENGINE_STATE_PAUSED:
            session_state = PLAY_CONTROLLER_STATE_PAUSED;
            break;

        case PLAY_ENGINE_STATE_ERROR:
            session_state = PLAY_CONTROLLER_STATE_ERROR;
            break;

        case PLAY_ENGINE_STATE_READY:
        case PLAY_ENGINE_STATE_STOPPED:
        default:
            if (session_state == PLAY_CONTROLLER_STATE_PLAYING)
            {
                session_state = PLAY_CONTROLLER_STATE_READY;
            }
            break;
    }
}

void play_controller_get_view(play_controller_view_t *view)
{
    uint32_t played;
    uint32_t total;

    if (view == NULL)
    {
        return;
    }

    played = play_engine_get_played_samples();
    total = play_engine_get_total_samples();

    view->filename = session_filename;
    view->full_path = session_full_path;
    view->format = session_format;
    view->play_mode = session_play_mode;
    view->invert_signal = session_invert_signal;
    view->state = session_state;
    view->error_text = play_engine_get_error_text();
    view->output_pin = play_engine_get_output_pin();
    view->played_samples = played;
    view->total_samples = total;
    view->progress_percent = play_controller_percent(played, total);
    view->buffer_fill_percent = play_engine_get_buffer_fill_percent();
    view->jitter_ticks = play_engine_get_jitter_ticks();
    view->timing_marker_pin = WAV_PLAYBACK_TIMING_MARKER_ENABLE ?
        WAV_PLAYBACK_TIMING_MARKER_PIN : 0U;
}

const char* play_controller_get_filename(void)
{
    return session_filename;
}

const char* play_controller_get_full_path(void)
{
    return session_full_path;
}

file_format_t play_controller_get_format(void)
{
    return session_format;
}

menu_play_mode_t play_controller_get_play_mode(void)
{
    return session_play_mode;
}

bool play_controller_get_invert_signal(void)
{
    return session_invert_signal;
}

play_controller_state_t play_controller_get_state(void)
{
    return session_state;
}
