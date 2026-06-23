#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "play_screen.h"

#include "../drivers/lcd.h"

#define PLAY_SCREEN_NAME_MAX 64

static char selected_filename[PLAY_SCREEN_NAME_MAX];

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");

    lcd_set_cursor(0, row);
    lcd_print(line);
}

void play_screen_init(const char *filename)
{
    if (filename == NULL)
    {
        selected_filename[0] = '\0';
        return;
    }

    strncpy(selected_filename, filename, sizeof(selected_filename) - 1);
    selected_filename[sizeof(selected_filename) - 1] = '\0';
}

play_screen_action_t play_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            return PLAY_SCREEN_ACTION_BACK;

        default:
            break;
    }

    return PLAY_SCREEN_ACTION_NONE;
}

void play_screen_render(void)
{
    char line0[17];

    snprintf(line0, sizeof(line0), "%-16s", selected_filename);

    lcd_print_fixed(0, line0);
    lcd_print_fixed(1, "READY");
}

const char* play_screen_get_filename(void)
{
    return selected_filename;
}
