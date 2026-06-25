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
#include "ui/record_menu.h"
#include "ui/play_screen.h"
#include "ui/record_screen.h"
#include "record/record_engine.h"

#define CALIBRATION_PRESSED_THRESHOLD 900
#define CALIBRATION_RELEASED_THRESHOLD 900
#define FORCE_CALIBRATION_RIGHT_THRESHOLD 50
#define ACTIVE_KEYPAD_POLL_MS 5U
#define ACTIVE_LCD_UPDATE_MS 250U

typedef enum
{
    APP_SCREEN_BROWSER = 0,
    APP_SCREEN_PLAY,
    APP_SCREEN_RECORD,
    APP_SCREEN_PLAY_MENU,
    APP_SCREEN_RECORD_MENU
} app_screen_t;

static uint32_t last_lcd_update_ms = 0U;
static uint32_t last_active_keypad_poll_ms = 0U;
static bool sd_ok = false;
static app_screen_t current_screen = APP_SCREEN_BROWSER;

/* Fixed root-level destination for WAV, LEP and L16 recordings. */
static const char RECORDINGS_DIRECTORY[] = "/RECORDINGS";

static void lcd_print_line(uint8_t row, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

static uint16_t adc_average(uint8_t samples)
{
    uint32_t sum = 0UL;
    for (uint8_t i = 0U; i < samples; ++i)
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
    while (keypad_read_raw() < CALIBRATION_RELEASED_THRESHOLD) delay(20);
    delay(200);
}

static uint16_t calibrate_none(void)
{
    lcd_clear();
    lcd_print_line(0, "CAL NONE");
    lcd_print_line(1, "RELEASE ALL");
    while (keypad_read_raw() < CALIBRATION_RELEASED_THRESHOLD) delay(20);
    delay(500);
    return adc_average(32);
}

static uint16_t calibrate_button(const char *name)
{
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "PRESS %-10s", name);
    lcd_clear();
    lcd_print_line(0, line0);
    lcd_print_line(1, "WAIT INPUT");
    while (keypad_read_raw() > CALIBRATION_PRESSED_THRESHOLD) delay(20);
    delay(250);
    uint16_t value = adc_average(32);
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
    if (calibration == NULL) return false;
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
    if (!keypad_calibration_is_valid(calibration)) return false;
    return calibration_store_save_keypad(calibration);
}

static void app_enter_browser(void)
{
    browser_end_record_scratch();
    browser_restore_saved_position();
    current_screen = APP_SCREEN_BROWSER;
    lcd_clear();
}

static void app_enter_play(const char *filename, const char *full_path)
{
    browser_save_position();
    play_controller_start_session(filename, full_path,
                                  menu_get_play_mode(),
                                  menu_get_invert_signal(),
                                  menu_get_play_control_mode());
    current_screen = APP_SCREEN_PLAY;
    lcd_clear();
}

static void app_enter_record(void)
{
    record_engine_config_t config;
    browser_save_position();
    browser_begin_record_scratch();
    config.format = record_menu_get_format();
    config.wav_sample_rate = record_menu_get_wav_sample_rate();
    config.control_mode = record_menu_get_control_mode();
    (void)record_engine_start(RECORDINGS_DIRECTORY, &config);
    current_screen = APP_SCREEN_RECORD;
    lcd_clear();
}

static void app_handle_browser_event(button_event_t event)
{
    browser_action_t action = browser_handle_event(event);
    switch (action)
    {
        case BROWSER_ACTION_FILE_SELECTED:
            app_enter_play(browser_get_selected_name(), browser_get_selected_full_path());
            break;
        case BROWSER_ACTION_RECORD_REQUESTED:
            app_enter_record();
            break;
        case BROWSER_ACTION_PLAY_MENU_REQUESTED:
            browser_save_position();
            current_screen = APP_SCREEN_PLAY_MENU;
            lcd_clear();
            break;
        case BROWSER_ACTION_RECORD_MENU_REQUESTED:
            browser_save_position();
            current_screen = APP_SCREEN_RECORD_MENU;
            lcd_clear();
            break;
        default:
            break;
    }
}

static void app_handle_play_event(button_event_t event)
{
    switch (play_screen_handle_event(event))
    {
        case PLAY_SCREEN_ACTION_TOGGLE_PLAY:
            play_controller_toggle_play_pause();
            break;
        case PLAY_SCREEN_ACTION_BACK:
            play_controller_stop();
            app_enter_browser();
            break;
        default:
            break;
    }
}

static void app_handle_record_event(button_event_t event)
{
    record_screen_action_t action = record_screen_handle_event(event);
    record_engine_state_t state = record_engine_get_state();

    if (action == RECORD_SCREEN_ACTION_TOGGLE_PAUSE)
    {
        if ((state == RECORD_ENGINE_ARMED) ||
            (state == RECORD_ENGINE_RECORDING) ||
            (state == RECORD_ENGINE_PAUSED))
        {
            record_engine_toggle_pause();
        }
        return;
    }
    if (action == RECORD_SCREEN_ACTION_STOP_SAVE)
    {
        if ((state == RECORD_ENGINE_ARMED) || (state == RECORD_ENGINE_RECORDING) ||
            (state == RECORD_ENGINE_PAUSED))
        {
            record_engine_request_stop();
        }
        else if ((state == RECORD_ENGINE_FINISHED) ||
                 (state == RECORD_ENGINE_CANCELLED) ||
                 (state == RECORD_ENGINE_ERROR) ||
                 (state == RECORD_ENGINE_STOPPED))
        {
            browser_refresh();
            app_enter_browser();
        }
        return;
    }
    if (action == RECORD_SCREEN_ACTION_CANCEL_BACK)
    {
        if ((state == RECORD_ENGINE_ARMED) || (state == RECORD_ENGINE_RECORDING) ||
            (state == RECORD_ENGINE_PAUSED) || (state == RECORD_ENGINE_FINALIZING) ||
            (state == RECORD_ENGINE_ERROR))
        {
            /* Keep the cancellation/deletion confirmation on the RECORD screen. */
            record_engine_cancel();
            lcd_clear();
        }
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
    if (calibration_loaded && !keypad_calibration_is_valid(&calibration)) calibration_loaded = false;
    if (!calibration_loaded || force_calibration_requested())
    {
        if (!run_keypad_calibration(&calibration)) keypad_get_default_calibration(&calibration);
    }
    keypad_set_calibration(&calibration);

    browser_init(sd_ok);
    menu_init();
    record_menu_init();
    play_controller_init();
    record_engine_init();
    current_screen = APP_SCREEN_BROWSER;
    lcd_clear();
}

void loop()
{
    uint32_t now = millis();

    if (current_screen == APP_SCREEN_PLAY)
    {
        play_controller_service();
        if ((now - last_active_keypad_poll_ms) >= ACTIVE_KEYPAD_POLL_MS)
        {
            last_active_keypad_poll_ms = now;
            button_event_t event = keypad_get_event();
            if (event != BUTTON_EVENT_NONE) app_handle_play_event(event);
        }
        play_controller_service();
        if ((now - last_lcd_update_ms) >= ACTIVE_LCD_UPDATE_MS)
        {
            play_controller_view_t view;
            last_lcd_update_ms = now;
            play_controller_get_view(&view);
            play_screen_render(&view);
        }
        play_controller_service();
        return;
    }

    if (current_screen == APP_SCREEN_RECORD)
    {
        record_engine_service();
        if ((now - last_active_keypad_poll_ms) >= ACTIVE_KEYPAD_POLL_MS)
        {
            last_active_keypad_poll_ms = now;
            button_event_t event = keypad_get_event();
            if (event != BUTTON_EVENT_NONE) app_handle_record_event(event);
        }
        record_engine_service();
        if ((now - last_lcd_update_ms) >= ACTIVE_LCD_UPDATE_MS)
        {
            last_lcd_update_ms = now;
            record_screen_render();
        }
        record_engine_service();
        return;
    }

    browser_service();
    button_event_t event = keypad_get_event();
    if (event != BUTTON_EVENT_NONE)
    {
        if (current_screen == APP_SCREEN_BROWSER)
        {
            app_handle_browser_event(event);
        }
        else if (current_screen == APP_SCREEN_PLAY_MENU)
        {
            if (menu_handle_event(event) == MENU_ACTION_BACK) app_enter_browser();
        }
        else if (current_screen == APP_SCREEN_RECORD_MENU)
        {
            if (record_menu_handle_event(event) == RECORD_MENU_ACTION_BACK) app_enter_browser();
        }
    }

    if ((now - last_lcd_update_ms) >= 120U)
    {
        last_lcd_update_ms = now;
        switch (current_screen)
        {
            case APP_SCREEN_BROWSER: browser_render(); break;
            case APP_SCREEN_PLAY_MENU: menu_render(); break;
            case APP_SCREEN_RECORD_MENU: record_menu_render(); break;
            case APP_SCREEN_PLAY:
            {
                play_controller_view_t view;
                play_controller_get_view(&view);
                play_screen_render(&view);
                break;
            }
            case APP_SCREEN_RECORD: record_screen_render(); break;
            default: app_enter_browser(); break;
        }
    }
}
