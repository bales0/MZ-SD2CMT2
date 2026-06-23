#pragma once

#include <Arduino.h>
#include "../formats/file_format.h"
#include "../ui/menu.h"

typedef enum
{
    PLAY_ENGINE_STATE_STOPPED = 0,
    PLAY_ENGINE_STATE_READY,
    PLAY_ENGINE_STATE_RUNNING,
    PLAY_ENGINE_STATE_PAUSED,
    PLAY_ENGINE_STATE_ERROR
} play_engine_state_t;

typedef struct
{
    const char *full_path;
    file_format_t format;
    menu_play_mode_t play_mode;
    bool invert_signal;
} play_engine_config_t;

void play_engine_init(void);
bool play_engine_prepare(const play_engine_config_t *config);
bool play_engine_start(void);
bool play_engine_pause(void);
bool play_engine_resume(void);
void play_engine_stop(void);
play_engine_state_t play_engine_get_state(void);
