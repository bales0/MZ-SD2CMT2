#pragma once

#include <Arduino.h>
#include "../drivers/keypad.h"

typedef enum
{
    MENU_ACTION_NONE = 0,
    MENU_ACTION_BACK
} menu_action_t;

typedef enum
{
    MENU_PLAY_MODE_NORMAL = 0,
    MENU_PLAY_MODE_ULTRA_TURBO
} menu_play_mode_t;

void menu_init(void);
menu_action_t menu_handle_event(button_event_t event);
void menu_render(void);
menu_play_mode_t menu_get_play_mode(void);
bool menu_get_invert_signal(void);
