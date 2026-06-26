#include "record_menu.h"

#include <Arduino.h>

#include "../drivers/lcd.h"
#include "../drivers/flash_text.h"

typedef enum
{
    RECORD_MENU_ITEM_TYPE = 0,
    RECORD_MENU_ITEM_CONTROL,
    RECORD_MENU_ITEM_COUNT
} record_menu_item_t;

typedef enum
{
    /* Menu order: default WAV 44 kHz -> WAV 22 kHz -> L16 -> LEP. */
    RECORD_TYPE_WAV_44K = 0,
    RECORD_TYPE_WAV_22K,
    RECORD_TYPE_L16,
    RECORD_TYPE_LEP,
    RECORD_TYPE_COUNT
} record_type_t;

static record_menu_item_t selected_item = RECORD_MENU_ITEM_TYPE;
static record_type_t record_type = RECORD_TYPE_WAV_44K;
static record_control_mode_t control_mode = RECORD_CONTROL_MOTOR;

static const char text_rec_type[] PROGMEM = ">REC TYPE";
static const char text_rec_mode[] PROGMEM = ">REC MODE";
static const char text_motor[] PROGMEM = "MOTOR";
static const char text_auto[] PROGMEM = "AUTO";
static const char text_manual[] PROGMEM = "MANUAL";
static const char text_lep[] PROGMEM = "LEP 50us";
static const char text_l16[] PROGMEM = "L16 16us";
static const char text_wav22[] PROGMEM = "WAV 22kHz";
static const char text_wav44[] PROGMEM = "WAV 44kHz";
static const char text_unknown[] PROGMEM = "?";

static void lcd_line_P(uint8_t row, PGM_P text)
{
    char line[17];
    uint8_t length;
    flash_text_copy(line, sizeof(line), text);
    for (length = 0U; (length < 16U) && (line[length] != '\0'); ++length) {}
    while (length < 16U) line[length++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0U, row);
    lcd_print(line);
}

static void lcd_value_line_P(PGM_P text)
{
    char line[17];
    line[0] = ' ';
    flash_text_copy(&line[1], sizeof(line) - 1U, text);
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        if (line[i] == '\0')
        {
            for (; i < 16U; ++i) line[i] = ' ';
            break;
        }
    }
    line[16] = '\0';
    lcd_set_cursor(0U, 1U);
    lcd_print(line);
}

PGM_P record_control_mode_label_P(record_control_mode_t mode)
{
    switch (mode)
    {
        case RECORD_CONTROL_AUTO: return text_auto;
        case RECORD_CONTROL_MANUAL: return text_manual;
        case RECORD_CONTROL_MOTOR:
        default: return text_motor;
    }
}

static PGM_P record_type_label_P(void)
{
    switch (record_type)
    {
        case RECORD_TYPE_WAV_44K: return text_wav44;
        case RECORD_TYPE_WAV_22K: return text_wav22;
        case RECORD_TYPE_L16: return text_l16;
        case RECORD_TYPE_LEP: return text_lep;
        default: return text_unknown;
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
    record_type = RECORD_TYPE_WAV_44K;
    control_mode = RECORD_CONTROL_MOTOR;
}

record_menu_action_t record_menu_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
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
    if (selected_item == RECORD_MENU_ITEM_TYPE)
    {
        lcd_line_P(0U, text_rec_type);
        lcd_value_line_P(record_type_label_P());
        return;
    }
    lcd_line_P(0U, text_rec_mode);
    lcd_value_line_P(record_control_mode_label_P(control_mode));
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
