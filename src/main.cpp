#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "drivers/lcd.h"
#include "drivers/keypad.h"
#include "drivers/calibration_store.h"
#include "drivers/sdcard.h"

#include "ui/browser.h"

#define CALIBRATION_PRESSED_THRESHOLD 900
#define CALIBRATION_RELEASED_THRESHOLD 900
#define FORCE_CALIBRATION_RIGHT_THRESHOLD 50

static uint32_t last_lcd_update_ms = 0;

static bool sd_ok = false;

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
    if (keypad_read_raw() <= FORCE_CALIBRATION_RIGHT_THRESHOLD)
    {
        return true;
    }

    return false;
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

void setup()
{
    sdcard_early_prepare_pins();

    sd_ok = sdcard_init();

    lcd_init();
    keypad_init();

    lcd_clear();

    keypad_calibration_t calibration;

    bool calibration_loaded = calibration_store_load_keypad(&calibration);

    if (calibration_loaded)
    {
        if (!keypad_calibration_is_valid(&calibration))
        {
            calibration_loaded = false;
        }
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
    else
    {
        lcd_set_cursor(0, 0);

        if (sd_ok)
        {
            lcd_print("SD EARLY OK    ");
        }
        else
        {
            lcd_print("SD EARLY FAIL  ");
        }

        lcd_set_cursor(0, 1);
        lcd_print("DIR BROWSER    ");

        delay(1200);
    }

    keypad_set_calibration(&calibration);

    lcd_clear();
    lcd_print_line(0, "DIR LOAD");
    lcd_print_line(1, "PLEASE WAIT");

    browser_init(sd_ok);

    delay(700);

    lcd_clear();
}

void loop()
{
    button_event_t event = keypad_get_event();

    if (event != BUTTON_EVENT_NONE)
    {
        browser_handle_event(event);
    }

    uint32_t now = millis();

    if ((now - last_lcd_update_ms) >= 120)
    {
        last_lcd_update_ms = now;

        browser_render();
    }
}