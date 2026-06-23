#pragma once

#include <Arduino.h>
#include "../formats/file_format.h"
#include "../ui/menu.h"

#define PLAY_CONTROLLER_NAME_MAX 64
#define PLAY_CONTROLLER_PATH_MAX 160

typedef enum
{
    PLAY_CONTROLLER_STATE_READY = 0,
    PLAY_CONTROLLER_STATE_PLAYING,
    PLAY_CONTROLLER_STATE_PAUSED,
    PLAY_CONTROLLER_STATE_ERROR
} play_controller_state_t;

typedef struct
{
    const char *filename;
    const char *full_path;
    file_format_t format;
    menu_play_mode_t play_mode;
    bool invert_signal;
    play_controller_state_t state;
} play_controller_view_t;

void play_controller_init(void);
void play_controller_start_session(const char *filename, const char *full_path, menu_play_mode_t play_mode, bool invert_signal);
void play_controller_toggle_play_pause(void);
void play_controller_stop(void);
void play_controller_get_view(play_controller_view_t *view);
const char* play_controller_get_filename(void);
const char* play_controller_get_full_path(void);
file_format_t play_controller_get_format(void);
menu_play_mode_t play_controller_get_play_mode(void);
bool play_controller_get_invert_signal(void);
play_controller_state_t play_controller_get_state(void);
