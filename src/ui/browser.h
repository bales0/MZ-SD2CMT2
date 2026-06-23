#pragma once

#include <Arduino.h>

#include "../drivers/keypad.h"

void browser_init(bool initial_sd_ok);

void browser_handle_event(button_event_t event);
void browser_render(void);

const char* browser_get_selected_name(void);
bool browser_selected_is_directory(void);