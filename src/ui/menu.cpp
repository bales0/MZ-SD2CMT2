#include <Arduino.h>

#include "menu.h"
#include "../drivers/lcd.h"
#include "../drivers/flash_text.h"

typedef enum
{
    MENU_ITEM_INVERT_SIGNAL = 0,
    MENU_ITEM_PLAY_CONTROL,
    MENU_ITEM_LOADER,
    MENU_ITEM_SPEED,
    MENU_ITEM_COUNT
} menu_item_t;

typedef enum
{
    MENU_LOADER_NORMAL = 0,
    MENU_LOADER_AUTO,
    MENU_LOADER_UL,
    MENU_LOADER_UL_MZ800,
    MENU_LOADER_UL_MZ700,
    MENU_LOADER_MZ700,
    MENU_LOADER_IC,
    MENU_LOADER_TC,
    MENU_LOADER_COUNT
} menu_loader_t;

typedef enum
{
    MENU_SPEED_1_1 = 0,
    MENU_SPEED_1_2,
    MENU_SPEED_1_3,
    MENU_SPEED_1_4,
    MENU_SPEED_COUNT
} menu_speed_t;

static menu_item_t selected_item = MENU_ITEM_INVERT_SIGNAL;
static bool invert_signal = false;
static menu_loader_t loader_mode = MENU_LOADER_NORMAL;
static menu_speed_t loader_speed = MENU_SPEED_1_1;
static play_control_mode_t play_control_mode = PLAY_CONTROL_MOTOR;

static const char text_invert[] PROGMEM = ">WAV INVERT";
static const char text_play_control[] PROGMEM = ">PLAY CTRL";
static const char text_loader[] PROGMEM = ">LOADER";
static const char text_speed[] PROGMEM = ">SPEED";
static const char text_on[] PROGMEM = " ON";
static const char text_off[] PROGMEM = " OFF";
static const char text_normal[] PROGMEM = " NORMAL";
static const char text_auto[] PROGMEM = " AUTO";
static const char text_ul[] PROGMEM = " UL";
static const char text_ul_mz800[] PROGMEM = " UL MZ800";
static const char text_ul_mz700[] PROGMEM = " UL MZ700";
static const char text_mz700[] PROGMEM = " MZ700";
static const char text_ic[] PROGMEM = " IC";
static const char text_tc[] PROGMEM = " TC";
static const char text_speed_none[] PROGMEM = " --";
static const char text_speed_1_1[] PROGMEM = " 1:1";
static const char text_speed_1_4[] PROGMEM = " 1:4";
static const char text_speed_1_3[] PROGMEM = " 1:3";
static const char text_speed_1_2[] PROGMEM = " 1:2";
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

static bool loader_uses_speed(void)
{
    return (loader_mode == MENU_LOADER_NORMAL) ||
           (loader_mode == MENU_LOADER_MZ700) ||
           (loader_mode == MENU_LOADER_IC) ||
           (loader_mode == MENU_LOADER_TC);
}

static void normalize_speed(void)
{
    if ((loader_mode == MENU_LOADER_NORMAL) &&
        (loader_speed == MENU_SPEED_1_4))
    {
        loader_speed = MENU_SPEED_1_1;
    }
    else if ((loader_mode == MENU_LOADER_MZ700) &&
             (loader_speed != MENU_SPEED_1_1) &&
             (loader_speed != MENU_SPEED_1_3))
    {
        loader_speed = MENU_SPEED_1_1;
    }
    else if ((loader_mode == MENU_LOADER_IC) &&
             (loader_speed == MENU_SPEED_1_1))
    {
        loader_speed = MENU_SPEED_1_4;
    }
    else if ((loader_mode == MENU_LOADER_TC) &&
             ((loader_speed == MENU_SPEED_1_1) ||
              (loader_speed == MENU_SPEED_1_4)))
    {
        loader_speed = MENU_SPEED_1_3;
    }
}

static PGM_P loader_label_P(void)
{
    switch (loader_mode)
    {
        case MENU_LOADER_AUTO: return text_auto;
        case MENU_LOADER_UL: return text_ul;
        case MENU_LOADER_UL_MZ800: return text_ul_mz800;
        case MENU_LOADER_UL_MZ700: return text_ul_mz700;
        case MENU_LOADER_MZ700: return text_mz700;
        case MENU_LOADER_IC: return text_ic;
        case MENU_LOADER_TC: return text_tc;
        default: return text_normal;
    }
}

static PGM_P speed_label_P(void)
{
    if (!loader_uses_speed()) return text_speed_none;
    switch (loader_speed)
    {
        case MENU_SPEED_1_1: return text_speed_1_1;
        case MENU_SPEED_1_4: return text_speed_1_4;
        case MENU_SPEED_1_2: return text_speed_1_2;
        default: return text_speed_1_3;
    }
}

static void cycle_loader(void)
{
    loader_mode = (menu_loader_t)(((uint8_t)loader_mode + 1U) %
                                  (uint8_t)MENU_LOADER_COUNT);
    normalize_speed();
}

static void cycle_speed(void)
{
    if (loader_mode == MENU_LOADER_NORMAL)
    {
        if (loader_speed == MENU_SPEED_1_1)
        {
            loader_speed = MENU_SPEED_1_2;
        }
        else if (loader_speed == MENU_SPEED_1_2)
        {
            loader_speed = MENU_SPEED_1_3;
        }
        else
        {
            loader_speed = MENU_SPEED_1_1;
        }
    }
    else if (loader_mode == MENU_LOADER_MZ700)
    {
        loader_speed = (loader_speed == MENU_SPEED_1_1) ?
            MENU_SPEED_1_3 : MENU_SPEED_1_1;
    }
    else if (loader_mode == MENU_LOADER_IC)
    {
        if (loader_speed == MENU_SPEED_1_4)
        {
            loader_speed = MENU_SPEED_1_3;
        }
        else if (loader_speed == MENU_SPEED_1_3)
        {
            loader_speed = MENU_SPEED_1_2;
        }
        else
        {
            loader_speed = MENU_SPEED_1_4;
        }
    }
    else if (loader_mode == MENU_LOADER_TC)
    {
        loader_speed = (loader_speed == MENU_SPEED_1_3) ?
            MENU_SPEED_1_2 : MENU_SPEED_1_3;
    }
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
    loader_mode = MENU_LOADER_NORMAL;
    loader_speed = MENU_SPEED_1_1;
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
            {
                invert_signal = !invert_signal;
            }
            else if (selected_item == MENU_ITEM_PLAY_CONTROL)
            {
                play_control_mode = (play_control_mode == PLAY_CONTROL_MOTOR) ?
                    PLAY_CONTROL_MANUAL : PLAY_CONTROL_MOTOR;
            }
            else if (selected_item == MENU_ITEM_LOADER)
            {
                cycle_loader();
            }
            else
            {
                cycle_speed();
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
    if (selected_item == MENU_ITEM_INVERT_SIGNAL)
    {
        lcd_print_fixed_P(0U, text_invert);
        lcd_print_fixed_P(1U, invert_signal ? text_on : text_off);
        return;
    }

    if (selected_item == MENU_ITEM_PLAY_CONTROL)
    {
        lcd_print_fixed_P(0U, text_play_control);
        lcd_print_fixed_P(1U, play_control_mode == PLAY_CONTROL_MANUAL ?
                          text_manual : text_motor);
        return;
    }

    if (selected_item == MENU_ITEM_LOADER)
    {
        lcd_print_fixed_P(0U, text_loader);
        lcd_print_fixed_P(1U, loader_label_P());
        return;
    }

    lcd_print_fixed_P(0U, text_speed);
    lcd_print_fixed_P(1U, speed_label_P());
}

bool menu_get_invert_signal(void) { return invert_signal; }

loader_mode_t menu_get_loader_mode(void)
{
    switch (loader_mode)
    {
        case MENU_LOADER_NORMAL:
            if (loader_speed == MENU_SPEED_1_2)
                return LOADER_MODE_NORMAL_1_2;
            if (loader_speed == MENU_SPEED_1_3)
                return LOADER_MODE_NORMAL_1_3;
            return LOADER_MODE_NORMAL_1_1;
        case MENU_LOADER_AUTO:
            return LOADER_MODE_AUTO;
        case MENU_LOADER_UL:
            return LOADER_MODE_UL;
        case MENU_LOADER_UL_MZ800:
            return LOADER_MODE_UL_MZ800;
        case MENU_LOADER_UL_MZ700:
            return LOADER_MODE_UL_MZ700;
        case MENU_LOADER_MZ700:
            return (loader_speed == MENU_SPEED_1_3) ?
                LOADER_MODE_MZ700_3X : LOADER_MODE_MZ700_1X;
        case MENU_LOADER_IC:
            if (loader_speed == MENU_SPEED_1_2) return LOADER_MODE_IC_1_2;
            if (loader_speed == MENU_SPEED_1_3) return LOADER_MODE_IC_1_3;
            return LOADER_MODE_IC_1_4;
        case MENU_LOADER_TC:
            return (loader_speed == MENU_SPEED_1_2) ?
                LOADER_MODE_TC_1_2 : LOADER_MODE_TC_1_3;
        default:
            return LOADER_MODE_NORMAL_1_1;
    }
}

play_control_mode_t menu_get_play_control_mode(void) { return play_control_mode; }
