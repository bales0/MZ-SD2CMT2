#pragma once

#include <Arduino.h>

#include "../drivers/keypad.h"

typedef enum
{
    PLAY_SCREEN_ACTION_NONE = 0,
    PLAY_SCREEN_ACTION_BACK
} play_screen_action_t;

void play_screen_init(const char *filename);

play_screen_action_t play_screen_handle_event(button_event_t event);
void play_screen_render(void);

const char* play_screen_get_filename(void);
