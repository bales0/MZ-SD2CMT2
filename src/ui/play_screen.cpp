#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "play_screen.h"

#include "../drivers/lcd.h"

#define PLAY_SCREEN_NAME_MAX 64
#define PLAY_SCREEN_PATH_MAX 160

static char selected_filename[PLAY_SCREEN_NAME_MAX];
static char selected_full_path[PLAY_SCREEN_PATH_MAX];
static file_format_t selected_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t selected_play_mode = MENU_PLAY_MODE_NORMAL;
static bool selected_invert_signal = false;
static play_screen_state_t play_state = PLAY_SCREEN_STATE_READY;

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

static const char* play_mode_to_short_label(menu_play_mode_t play_mode)
{
    if (play_mode == MENU_PLAY_MODE_ULTRA_TURBO)
    {
        return "ULTR";
    }
    return "NORM";
}

static const char* state_to_label(play_screen_state_t state)
{
    switch (state)
    {
        case PLAY_SCREEN_STATE_PLAYING: return "PLY";
        case PLAY_SCREEN_STATE_PAUSED: return "PAU";
        case PLAY_SCREEN_STATE_READY:
        default: return "RDY";
    }
}

static bool play_screen_can_start(void)
{
    return selected_format != FILE_FORMAT_UNKNOWN;
}

static void play_screen_toggle_play_pause(void)
{
    if (!play_screen_can_start())
    {
        return;
    }

    switch (play_state)
    {
        case PLAY_SCREEN_STATE_READY:
            play_state = PLAY_SCREEN_STATE_PLAYING;
            break;
        case PLAY_SCREEN_STATE_PLAYING:
            play_state = PLAY_SCREEN_STATE_PAUSED;
            break;
        case PLAY_SCREEN_STATE_PAUSED:
            play_state = PLAY_SCREEN_STATE_PLAYING;
            break;
        default:
            play_state = PLAY_SCREEN_STATE_READY;
            break;
    }
}

void play_screen_init(const char *filename, const char *full_path, menu_play_mode_t play_mode, bool invert_signal)
{
    if (filename == NULL)
    {
        selected_filename[0] = '\0';
    }
    else
    {
        strncpy(selected_filename, filename, sizeof(selected_filename) - 1);
        selected_filename[sizeof(selected_filename) - 1] = '\0';
    }

    if (full_path == NULL)
    {
        selected_full_path[0] = '\0';
    }
    else
    {
        strncpy(selected_full_path, full_path, sizeof(selected_full_path) - 1);
        selected_full_path[sizeof(selected_full_path) - 1] = '\0';
    }

    selected_play_mode = play_mode;
    selected_invert_signal = invert_signal;
    selected_format = file_format_detect_from_name(selected_filename);
    play_state = PLAY_SCREEN_STATE_READY;
}

play_screen_action_t play_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT_SHORT:
            play_screen_toggle_play_pause();
            return PLAY_SCREEN_ACTION_NONE;
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            play_state = PLAY_SCREEN_STATE_READY;
            return PLAY_SCREEN_ACTION_BACK;
        default:
            break;
    }
    return PLAY_SCREEN_ACTION_NONE;
}

void play_screen_render(void)
{
    char line1[17];
    lcd_print_fixed(0, selected_filename);

    if (selected_format == FILE_FORMAT_UNKNOWN)
    {
        snprintf(line1, sizeof(line1), "UNSUPPORTED");
    }
    else
    {
        snprintf(
            line1,
            sizeof(line1),
            "%s %s %s%s",
            state_to_label(play_state),
            file_format_to_label(selected_format),
            play_mode_to_short_label(selected_play_mode),
            selected_invert_signal ? " I" : ""
        );
    }

    lcd_print_fixed(1, line1);
}

const char* play_screen_get_filename(void)
{
    return selected_filename;
}

const char* play_screen_get_full_path(void)
{
    return selected_full_path;
}

file_format_t play_screen_get_file_format(void)
{
    return selected_format;
}

menu_play_mode_t play_screen_get_play_mode(void)
{
    return selected_play_mode;
}

bool play_screen_get_invert_signal(void)
{
    return selected_invert_signal;
}

play_screen_state_t play_screen_get_state(void)
{
    return play_state;
}
