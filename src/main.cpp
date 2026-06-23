#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "drivers/lcd.h"
#include "drivers/keypad.h"
#include "drivers/calibration_store.h"
#include "drivers/sdcard.h"

#include "play/play_controller.h"
#include "ui/browser.h"
#include "ui/menu.h"
#include "ui/play_screen.h"

#define CALIBRATION_PRESSED_THRESHOLD 900
#define CALIBRATION_RELEASED_THRESHOLD 900
#define FORCE_CALIBRATION_RIGHT_THRESHOLD 50

typedef enum
{
    APP_SCREEN_BROWSER = 0,
    APP_SCREEN_PLAY_STUB,
    APP_SCREEN_MENU
} app_screen_t;

static uint32_t last_lcd_update_ms = 0;
static bool sd_ok = false;
static app_screen_t current_screen = APP_SCREEN_BROWSER;

static void lcd_print_line(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text);
    lcd_set_cursor(0, row);
    lcd_print(line);
}

static uint16_t adc_average(uint8_t samples)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        sum += keypad_read_raw();
        delay(5);
    }
    return (uint16_t)(sum / samples);
}

static void wait_for_release(void)
{
    lcd_print_line(0, "RELEASE ALL");
    lcd_print_line(1, "WAIT...");
    while (keypad_read_raw() < CALIBRATION_RELEASED_THRESHOLD)
    {
        delay(20);
    }
    delay(200);
}

static uint16_t calibrate_none(void)
{
    lcd_clear();
    lcd_print_line(0, "CAL NONE");
    lcd_print_line(1, "RELEASE ALL");
    while (keypad_read_raw() < CALIBRATION_RELEASED_THRESHOLD)
    {
        delay(20);
    }
    delay(500);
    return adc_average(32);
}

static uint16_t calibrate_button(const char *name)
{
    char line0[17];
    snprintf(line0, sizeof(line0), "PRESS %-10s", name);
    lcd_clear();
    lcd_print_line(0, line0);
    lcd_print_line(1, "WAIT INPUT");
    while (keypad_read_raw() > CALIBRATION_PRESSED_THRESHOLD)
    {
        delay(20);
    }
    delay(250);
    uint16_t value = adc_average(32);
    char line1[17];
    snprintf(line1, sizeof(line1), "ADC:%4u", value);
    lcd_print_line(0, name);
    lcd_print_line(1, line1);
    delay(700);
    wait_for_release();
    return value;
}

static bool force_calibration_requested(void)
{
    return keypad_read_raw() <= FORCE_CALIBRATION_RIGHT_THRESHOLD;
}

static bool run_keypad_calibration(keypad_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return false;
    }

    lcd_clear();
    lcd_print_line(0, "KEYPAD");
    lcd_print_line(1, "CALIBRATION");
    delay(1200);

    calibration->none = calibrate_none();
    calibration->up = calibrate_button("UP");
    calibration->down = calibrate_button("DOWN");
    calibration->left = calibrate_button("LEFT");
    calibration->right = calibrate_button("RIGHT");
    calibration->select = calibrate_button("SELECT");

    if (!keypad_calibration_is_valid(calibration))
    {
        lcd_clear();
        lcd_print_line(0, "CAL INVALID");
        lcd_print_line(1, "NOT SAVED");
        delay(2000);
        return false;
    }

    lcd_clear();
    lcd_print_line(0, "SAVING");
    lcd_print_line(1, "EEPROM");
    bool saved = calibration_store_save_keypad(calibration);
    delay(800);
    lcd_clear();

    if (saved)
    {
        lcd_print_line(0, "CALIBRATION");
        lcd_print_line(1, "SAVED");
    }
    else
    {
        lcd_print_line(0, "CALIBRATION");
        lcd_print_line(1, "SAVE ERROR");
    }

    delay(1200);
    return saved;
}

static void app_enter_browser(void)
{
    browser_restore_saved_position();
    current_screen = APP_SCREEN_BROWSER;
    lcd_clear();
}

static void app_enter_play_stub(const char *filename, const char *full_path)
{
    browser_save_position();
    play_controller_start_session(filename, full_path, menu_get_play_mode(), menu_get_invert_signal());
    current_screen = APP_SCREEN_PLAY_STUB;
    lcd_clear();
}

static void app_enter_menu(void)
{
    browser_save_position();
    current_screen = APP_SCREEN_MENU;
    lcd_clear();
}

static void app_handle_browser_event(button_event_t event)
{
    browser_action_t action = browser_handle_event(event);
    switch (action)
    {
        case BROWSER_ACTION_FILE_SELECTED:
            app_enter_play_stub(browser_get_selected_name(), browser_get_selected_full_path());
            break;
        case BROWSER_ACTION_MENU_REQUESTED:
            app_enter_menu();
            break;
        case BROWSER_ACTION_NONE:
        default:
            break;
    }
}

static void app_handle_play_stub_event(button_event_t event)
{
    play_screen_action_t action = play_screen_handle_event(event);
    switch (action)
    {
        case PLAY_SCREEN_ACTION_TOGGLE_PLAY:
            play_controller_toggle_play_pause();
            break;
        case PLAY_SCREEN_ACTION_BACK:
            play_controller_stop();
            app_enter_browser();
            break;
        case PLAY_SCREEN_ACTION_NONE:
        default:
            break;
    }
}

static void app_handle_menu_event(button_event_t event)
{
    menu_action_t action = menu_handle_event(event);
    if (action == MENU_ACTION_BACK)
    {
        app_enter_browser();
    }
}

void setup()
{
    sdcard_early_prepare_pins();
    sd_ok = sdcard_init();

    lcd_init();
    keypad_init();
    lcd_clear();

    keypad_calibration_t calibration;
    bool calibration_loaded = calibration_store_load_keypad(&calibration);
    if (calibration_loaded && !keypad_calibration_is_valid(&calibration))
    {
        calibration_loaded = false;
    }

    bool force_calibration = force_calibration_requested();
    if (!calibration_loaded || force_calibration)
    {
        bool calibration_ok = run_keypad_calibration(&calibration);
        if (!calibration_ok)
        {
            keypad_get_default_calibration(&calibration);
            lcd_clear();
            lcd_print_line(0, "USING DEFAULT");
            lcd_print_line(1, "CALIBRATION");
            delay(1500);
        }
    }

    keypad_set_calibration(&calibration);

    browser_init(sd_ok);
    menu_init();
    play_controller_init();
    current_screen = APP_SCREEN_BROWSER;

    lcd_clear();
}

void loop()
{
    button_event_t event = keypad_get_event();
    if (event != BUTTON_EVENT_NONE)
    {
        switch (current_screen)
        {
            case APP_SCREEN_BROWSER:
                app_handle_browser_event(event);
                break;
            case APP_SCREEN_PLAY_STUB:
                app_handle_play_stub_event(event);
                break;
            case APP_SCREEN_MENU:
                app_handle_menu_event(event);
                break;
            default:
                app_enter_browser();
                break;
        }
    }

    uint32_t now = millis();
    if ((now - last_lcd_update_ms) >= 120)
    {
        last_lcd_update_ms = now;
        switch (current_screen)
        {
            case APP_SCREEN_BROWSER:
                browser_render();
                break;
            case APP_SCREEN_PLAY_STUB:
            {
                play_controller_view_t view;
                play_controller_get_view(&view);
                play_screen_render(&view);
                break;
            }
            case APP_SCREEN_MENU:
                menu_render();
                break;
            default:
                app_enter_browser();
                break;
        }
    }
}
