#include <Arduino.h>
#include <stdio.h>

#include "menu.h"
#include "../drivers/lcd.h"

typedef enum
{
    MENU_ITEM_PLAY_MODE = 0,
    MENU_ITEM_INVERT_SIGNAL,
    MENU_ITEM_PLAY_CONTROL,
    MENU_ITEM_COUNT
} menu_item_t;

static menu_item_t selected_item = MENU_ITEM_PLAY_MODE;
static menu_play_mode_t play_mode = MENU_PLAY_MODE_NORMAL;
static bool invert_signal = false;
static play_control_mode_t play_control_mode = PLAY_CONTROL_MOTOR;

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

const char *play_control_mode_label(play_control_mode_t mode)
{
    return (mode == PLAY_CONTROL_MANUAL) ? "MANUAL" : "MOTOR";
}

static void move_selection(int8_t direction)
{
    int8_t next = (int8_t)selected_item + direction;
    if (next < 0) next = (int8_t)MENU_ITEM_COUNT - 1;
    if (next >= (int8_t)MENU_ITEM_COUNT) next = 0;
    selected_item = (menu_item_t)next;
}

void menu_init(void)
{
    selected_item = MENU_ITEM_PLAY_MODE;
    play_mode = MENU_PLAY_MODE_NORMAL;
    invert_signal = false;
    play_control_mode = PLAY_CONTROL_MOTOR;
}

menu_action_t menu_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
            move_selection(-1);
            break;

        case BUTTON_EVENT_DOWN_PRESS:
        case BUTTON_EVENT_DOWN_REPEAT:
            move_selection(1);
            break;

        case BUTTON_EVENT_SELECT_SHORT:
            if (selected_item == MENU_ITEM_PLAY_MODE)
            {
                play_mode = (play_mode == MENU_PLAY_MODE_NORMAL) ?
                    MENU_PLAY_MODE_ULTRA_FAST : MENU_PLAY_MODE_NORMAL;
            }
            else if (selected_item == MENU_ITEM_INVERT_SIGNAL)
            {
                invert_signal = !invert_signal;
            }
            else
            {
                play_control_mode = (play_control_mode == PLAY_CONTROL_MOTOR) ?
                    PLAY_CONTROL_MANUAL : PLAY_CONTROL_MOTOR;
            }
            break;

        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            return MENU_ACTION_BACK;

        default:
            break;
    }
    return MENU_ACTION_NONE;
}

void menu_render(void)
{
    switch (selected_item)
    {
        case MENU_ITEM_PLAY_MODE:
            lcd_print_fixed(0, ">PLAY MODE");
            lcd_print_fixed(1, play_mode == MENU_PLAY_MODE_NORMAL ?
                            " NORMAL" : " ULTRA FAST");
            break;

        case MENU_ITEM_INVERT_SIGNAL:
            lcd_print_fixed(0, ">INVERT SIG.");
            lcd_print_fixed(1, invert_signal ? " ON" : " OFF");
            break;

        case MENU_ITEM_PLAY_CONTROL:
        default:
            lcd_print_fixed(0, ">PLAY CTRL");
            lcd_print_fixed(1, play_control_mode == PLAY_CONTROL_MANUAL ?
                            " MANUAL" : " MOTOR");
            break;
    }
}

menu_play_mode_t menu_get_play_mode(void) { return play_mode; }
bool menu_get_invert_signal(void) { return invert_signal; }
play_control_mode_t menu_get_play_control_mode(void) { return play_control_mode; }
