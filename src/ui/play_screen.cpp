#include <Arduino.h>
#include <stdio.h>
#include "play_screen.h"
#include "../drivers/lcd.h"
#include "../formats/file_format.h"

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

static const char* play_mode_to_short_label(menu_play_mode_t play_mode)
{
    return (play_mode == MENU_PLAY_MODE_ULTRA_TURBO) ? "ULTR" : "NORM";
}

static const char* state_to_label(play_controller_state_t state)
{
    switch (state)
    {
        case PLAY_CONTROLLER_STATE_PLAYING: return "PLY";
        case PLAY_CONTROLLER_STATE_PAUSED: return "PAU";
        case PLAY_CONTROLLER_STATE_ERROR: return "ERR";
        case PLAY_CONTROLLER_STATE_READY:
        default: return "RDY";
    }
}

play_screen_action_t play_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT_SHORT:
            return PLAY_SCREEN_ACTION_TOGGLE_PLAY;
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            return PLAY_SCREEN_ACTION_BACK;
        default:
            break;
    }
    return PLAY_SCREEN_ACTION_NONE;
}

void play_screen_render(const play_controller_view_t *view)
{
    char line1[17];
    if (view == NULL)
    {
        lcd_print_fixed(0, "PLAY");
        lcd_print_fixed(1, "NO SESSION");
        return;
    }

    lcd_print_fixed(0, view->filename);

    if (view->format == FILE_FORMAT_UNKNOWN)
    {
        snprintf(line1, sizeof(line1), "UNSUPPORTED");
    }
    else
    {
        snprintf(line1, sizeof(line1), "%s %s %s%s", state_to_label(view->state), file_format_to_label(view->format), play_mode_to_short_label(view->play_mode), view->invert_signal ? " I" : "");
    }
    lcd_print_fixed(1, line1);
}
