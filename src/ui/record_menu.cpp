#include "record_menu.h"

#include <Arduino.h>
#include <stdio.h>

#include "../drivers/lcd.h"

typedef enum
{
    RECORD_MENU_ITEM_TYPE = 0,
    RECORD_MENU_ITEM_CONTROL,
    RECORD_MENU_ITEM_COUNT
} record_menu_item_t;

typedef enum
{
    RECORD_TYPE_LEP = 0,
    RECORD_TYPE_L16,
    RECORD_TYPE_WAV_22K,
    RECORD_TYPE_WAV_44K,
    RECORD_TYPE_COUNT
} record_type_t;

static record_menu_item_t selected_item = RECORD_MENU_ITEM_TYPE;
/* Preserve the previously safe default: WAV at 22.05 kHz. */
static record_type_t record_type = RECORD_TYPE_WAV_22K;
static record_control_mode_t control_mode = RECORD_CONTROL_MOTOR;

static void lcd_line(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

const char *record_control_mode_label(record_control_mode_t mode)
{
    switch (mode)
    {
        case RECORD_CONTROL_MOTOR: return "MOTOR";
        case RECORD_CONTROL_AUTO: return "AUTO";
        case RECORD_CONTROL_MANUAL: return "MANUAL";
        default: return "?";
    }
}

static const char *record_type_label(void)
{
    switch (record_type)
    {
        case RECORD_TYPE_LEP: return "LEP 50us";
        case RECORD_TYPE_L16: return "L16 16us";
        case RECORD_TYPE_WAV_22K: return "WAV 22kHz";
        case RECORD_TYPE_WAV_44K: return "WAV 44kHz";
        default: return "?";
    }
}

static void toggle_current(void)
{
    if (selected_item == RECORD_MENU_ITEM_TYPE)
    {
        record_type = (record_type_t)(((uint8_t)record_type + 1U) %
                                      (uint8_t)RECORD_TYPE_COUNT);
    }
    else
    {
        control_mode = (control_mode == RECORD_CONTROL_MOTOR) ? RECORD_CONTROL_AUTO :
                       (control_mode == RECORD_CONTROL_AUTO) ? RECORD_CONTROL_MANUAL :
                                                               RECORD_CONTROL_MOTOR;
    }
}

void record_menu_init(void)
{
    selected_item = RECORD_MENU_ITEM_TYPE;
    record_type = RECORD_TYPE_WAV_22K;
    control_mode = RECORD_CONTROL_MOTOR;
}

record_menu_action_t record_menu_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
            selected_item = (selected_item == RECORD_MENU_ITEM_TYPE) ?
                RECORD_MENU_ITEM_CONTROL : RECORD_MENU_ITEM_TYPE;
            break;
        case BUTTON_EVENT_DOWN_PRESS:
        case BUTTON_EVENT_DOWN_REPEAT:
            selected_item = (selected_item == RECORD_MENU_ITEM_TYPE) ?
                RECORD_MENU_ITEM_CONTROL : RECORD_MENU_ITEM_TYPE;
            break;
        case BUTTON_EVENT_SELECT_SHORT:
            toggle_current();
            break;
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            return RECORD_MENU_ACTION_BACK;
        default:
            break;
    }
    return RECORD_MENU_ACTION_NONE;
}

void record_menu_render(void)
{
    char line[17];
    if (selected_item == RECORD_MENU_ITEM_TYPE)
    {
        lcd_line(0, ">REC TYPE");
        snprintf(line, sizeof(line), " %s", record_type_label());
        lcd_line(1, line);
    }
    else
    {
        lcd_line(0, ">REC MODE");
        snprintf(line, sizeof(line), " %s", record_control_mode_label(control_mode));
        lcd_line(1, line);
    }
}

file_format_t record_menu_get_format(void)
{
    return (record_type == RECORD_TYPE_LEP) ? FILE_FORMAT_LEP :
           (record_type == RECORD_TYPE_L16) ? FILE_FORMAT_L16 : FILE_FORMAT_WAV;
}

uint32_t record_menu_get_wav_sample_rate(void)
{
    return (record_type == RECORD_TYPE_WAV_44K) ? 44100UL : 22050UL;
}

record_control_mode_t record_menu_get_control_mode(void) { return control_mode; }
