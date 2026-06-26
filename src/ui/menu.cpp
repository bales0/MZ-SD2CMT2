#include <Arduino.h>

#include "menu.h"
#include "../drivers/lcd.h"
#include "../drivers/flash_text.h"

typedef enum
{
    MENU_ITEM_INVERT_SIGNAL = 0,
    MENU_ITEM_PLAY_CONTROL,
    MENU_ITEM_COUNT
} menu_item_t;

static menu_item_t selected_item = MENU_ITEM_INVERT_SIGNAL;
static bool invert_signal = false;
static play_control_mode_t play_control_mode = PLAY_CONTROL_MOTOR;

static const char text_invert[] PROGMEM = ">INVERT SIG.";
static const char text_play_control[] PROGMEM = ">PLAY CTRL";
static const char text_on[] PROGMEM = " ON";
static const char text_off[] PROGMEM = " OFF";
static const char text_motor[] PROGMEM = " MOTOR";
static const char text_manual[] PROGMEM = " MANUAL";
static const char text_motor_label[] PROGMEM = "MOTOR";
static const char text_manual_label[] PROGMEM = "MANUAL";

static void lcd_print_fixed_P(uint8_t row, PGM_P text)
{
    char line[17];
    uint8_t length;

    flash_text_copy(line, sizeof(line), text);
    for (length = 0U; (length < 16U) && (line[length] != '\0'); ++length) {}
    while (length < 16U) line[length++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, row);
    lcd_print(line);
}

PGM_P play_control_mode_label_P(play_control_mode_t mode)
{
    return (mode == PLAY_CONTROL_MANUAL) ? text_manual_label : text_motor_label;
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
    selected_item = MENU_ITEM_INVERT_SIGNAL;
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
            if (selected_item == MENU_ITEM_INVERT_SIGNAL)
                invert_signal = !invert_signal;
            else
                play_control_mode = (play_control_mode == PLAY_CONTROL_MOTOR) ?
                    PLAY_CONTROL_MANUAL : PLAY_CONTROL_MOTOR;
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
    if (selected_item == MENU_ITEM_INVERT_SIGNAL)
    {
        lcd_print_fixed_P(0U, text_invert);
        lcd_print_fixed_P(1U, invert_signal ? text_on : text_off);
        return;
    }

    lcd_print_fixed_P(0U, text_play_control);
    lcd_print_fixed_P(1U, play_control_mode == PLAY_CONTROL_MANUAL ?
                      text_manual : text_motor);
}

bool menu_get_invert_signal(void) { return invert_signal; }
play_control_mode_t menu_get_play_control_mode(void) { return play_control_mode; }
