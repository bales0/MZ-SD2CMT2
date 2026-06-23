#pragma once

#include <Arduino.h>
#include "../drivers/keypad.h"

typedef enum
{
    BROWSER_ACTION_NONE = 0,
    BROWSER_ACTION_FILE_SELECTED,
    BROWSER_ACTION_MENU_REQUESTED
} browser_action_t;

void browser_init(bool initial_sd_ok);
browser_action_t browser_handle_event(button_event_t event);
void browser_render(void);
const char* browser_get_selected_name(void);
const char* browser_get_selected_full_path(void);
bool browser_selected_is_directory(void);
void browser_save_position(void);
void browser_restore_saved_position(void);
