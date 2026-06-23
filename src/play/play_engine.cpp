#include <Arduino.h>
#include <string.h>
#include "play_engine.h"

#define PLAY_ENGINE_PATH_MAX 160

static char prepared_full_path[PLAY_ENGINE_PATH_MAX];
static file_format_t prepared_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t prepared_play_mode = MENU_PLAY_MODE_NORMAL;
static bool prepared_invert_signal = false;
static play_engine_state_t engine_state = PLAY_ENGINE_STATE_STOPPED;

void play_engine_init(void)
{
    prepared_full_path[0] = '\0';
    prepared_format = FILE_FORMAT_UNKNOWN;
    prepared_play_mode = MENU_PLAY_MODE_NORMAL;
    prepared_invert_signal = false;
    engine_state = PLAY_ENGINE_STATE_STOPPED;
}

bool play_engine_prepare(const play_engine_config_t *config)
{
    if ((config == NULL) || (config->full_path == NULL) || (config->format == FILE_FORMAT_UNKNOWN))
    {
        engine_state = PLAY_ENGINE_STATE_ERROR;
        return false;
    }

    strncpy(prepared_full_path, config->full_path, sizeof(prepared_full_path) - 1);
    prepared_full_path[sizeof(prepared_full_path) - 1] = '\0';
    prepared_format = config->format;
    prepared_play_mode = config->play_mode;
    prepared_invert_signal = config->invert_signal;
    engine_state = PLAY_ENGINE_STATE_READY;
    return true;
}

bool play_engine_start(void)
{
    if ((engine_state != PLAY_ENGINE_STATE_READY) && (engine_state != PLAY_ENGINE_STATE_PAUSED))
    {
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
    engine_state = PLAY_ENGINE_STATE_PAUSED;
    return true;
}

bool play_engine_resume(void)
{
    if (engine_state != PLAY_ENGINE_STATE_PAUSED)
    {
        return false;
    }
    engine_state = PLAY_ENGINE_STATE_RUNNING;
    return true;
}

void play_engine_stop(void)
{
    engine_state = PLAY_ENGINE_STATE_STOPPED;
}

play_engine_state_t play_engine_get_state(void)
{
    return engine_state;
}
