#include <Arduino.h>
#include <string.h>
#include "play_controller.h"
#include "play_engine.h"

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

void play_controller_start_session(const char *filename, const char *full_path, menu_play_mode_t play_mode, bool invert_signal)
{
    if (filename == NULL)
    {
        session_filename[0] = '\0';
    }
    else
    {
        strncpy(session_filename, filename, sizeof(session_filename) - 1);
        session_filename[sizeof(session_filename) - 1] = '\0';
    }

    if (full_path == NULL)
    {
        session_full_path[0] = '\0';
    }
    else
    {
        strncpy(session_full_path, full_path, sizeof(session_full_path) - 1);
        session_full_path[sizeof(session_full_path) - 1] = '\0';
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

void play_controller_get_view(play_controller_view_t *view)
{
    if (view == NULL)
    {
        return;
    }
    view->filename = session_filename;
    view->full_path = session_full_path;
    view->format = session_format;
    view->play_mode = session_play_mode;
    view->invert_signal = session_invert_signal;
    view->state = session_state;
}

const char* play_controller_get_filename(void) { return session_filename; }
const char* play_controller_get_full_path(void) { return session_full_path; }
file_format_t play_controller_get_format(void) { return session_format; }
menu_play_mode_t play_controller_get_play_mode(void) { return session_play_mode; }
bool play_controller_get_invert_signal(void) { return session_invert_signal; }
play_controller_state_t play_controller_get_state(void) { return session_state; }
