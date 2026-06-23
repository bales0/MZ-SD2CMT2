#pragma once

#include <Arduino.h>

#include "../drivers/keypad.h"
#include "../formats/file_format.h"
#include "menu.h"

typedef enum
{
    PLAY_SCREEN_ACTION_NONE = 0,
    PLAY_SCREEN_ACTION_BACK
} play_screen_action_t;

typedef enum
{
    PLAY_SCREEN_STATE_READY = 0,
    PLAY_SCREEN_STATE_PLAYING,
    PLAY_SCREEN_STATE_PAUSED
} play_screen_state_t;

void play_screen_init(
    const char *filename,
    const char *full_path,
    menu_play_mode_t play_mode,
    bool invert_signal
);

play_screen_action_t play_screen_handle_event(button_event_t event);
void play_screen_render(void);

const char* play_screen_get_filename(void);
const char* play_screen_get_full_path(void);
file_format_t play_screen_get_file_format(void);
menu_play_mode_t play_screen_get_play_mode(void);
bool play_screen_get_invert_signal(void);
play_screen_state_t play_screen_get_state(void);
