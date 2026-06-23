#include <Arduino.h>
#include <stdio.h>
#include "menu.h"
#include "../drivers/lcd.h"

typedef enum
{
    MENU_ITEM_PLAY_MODE = 0,
    MENU_ITEM_INVERT_SIGNAL,
    MENU_ITEM_COUNT
} menu_item_t;

static menu_item_t selected_item = MENU_ITEM_PLAY_MODE;
static menu_play_mode_t play_mode = MENU_PLAY_MODE_NORMAL;
static bool invert_signal = false;

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

static void menu_move_up(void)
{
    selected_item = (selected_item == MENU_ITEM_PLAY_MODE) ? MENU_ITEM_INVERT_SIGNAL : (menu_item_t)((int)selected_item - 1);
}

static void menu_move_down(void)
{
    selected_item = (menu_item_t)(((int)selected_item + 1) % (int)MENU_ITEM_COUNT);
}

static void menu_toggle_current(void)
{
    switch (selected_item)
    {
        case MENU_ITEM_PLAY_MODE:
            play_mode = (play_mode == MENU_PLAY_MODE_NORMAL) ? MENU_PLAY_MODE_ULTRA_TURBO : MENU_PLAY_MODE_NORMAL;
            break;
        case MENU_ITEM_INVERT_SIGNAL:
            invert_signal = !invert_signal;
            break;
        default:
            break;
    }
}

void menu_init(void)
{
    selected_item = MENU_ITEM_PLAY_MODE;
    play_mode = MENU_PLAY_MODE_NORMAL;
    invert_signal = false;
}

menu_action_t menu_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
            menu_move_up();
            return MENU_ACTION_NONE;
        case BUTTON_EVENT_DOWN_PRESS:
        case BUTTON_EVENT_DOWN_REPEAT:
            menu_move_down();
            return MENU_ACTION_NONE;
        case BUTTON_EVENT_SELECT_SHORT:
            menu_toggle_current();
            return MENU_ACTION_NONE;
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
            lcd_print_fixed(1, play_mode == MENU_PLAY_MODE_NORMAL ? " NORMAL" : " ULTRA TURBO");
            break;
        case MENU_ITEM_INVERT_SIGNAL:
            lcd_print_fixed(0, ">INVERT SIG.");
            lcd_print_fixed(1, invert_signal ? " ON" : " OFF");
            break;
        default:
            lcd_print_fixed(0, ">MENU");
            lcd_print_fixed(1, " ERROR");
            break;
    }
}

menu_play_mode_t menu_get_play_mode(void)
{
    return play_mode;
}

bool menu_get_invert_signal(void)
{
    return invert_signal;
}
