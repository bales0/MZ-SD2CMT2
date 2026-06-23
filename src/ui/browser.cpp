#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "browser.h"

#include "../drivers/lcd.h"
#include "../drivers/sdcard.h"

#define MAX_DIR_ENTRIES 999
#define BROWSER_PATH_MAX 96
#define MESSAGE_HOLD_MS 1200

static bool sd_ok = false;

static uint16_t dir_count = 0;
static uint16_t selected_index = 0;

static char current_path[BROWSER_PATH_MAX] = "/";
static sdcard_entry_t current_entry;

static char status_message[17];
static uint32_t status_message_until_ms = 0;

static void lcd_print_line(uint8_t row, const char *text)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16s", text);

    lcd_set_cursor(0, row);
    lcd_print(line);
}

static void show_message(const char *line0, const char *line1, uint16_t hold_ms)
{
    lcd_clear();
    lcd_print_line(0, line0);
    lcd_print_line(1, line1);

    delay(hold_ms);

    lcd_clear();
}

static void set_status_message(const char *message)
{
    snprintf(status_message, sizeof(status_message), "%-16s", message);
    status_message_until_ms = millis() + MESSAGE_HOLD_MS;
}

static bool browser_is_root(void)
{
    return strcmp(current_path, "/") == 0;
}

static void browser_load_current_entry(void)
{
    memset(&current_entry, 0, sizeof(current_entry));

    if (!sd_ok)
    {
        strncpy(current_entry.name, sdcard_last_error(), sizeof(current_entry.name) - 1);
        current_entry.name[sizeof(current_entry.name) - 1] = '\0';
        current_entry.is_dir = false;
        current_entry.size = 0;

        return;
    }

    if (dir_count == 0)
    {
        strncpy(current_entry.name, "EMPTY", sizeof(current_entry.name) - 1);
        current_entry.name[sizeof(current_entry.name) - 1] = '\0';
        current_entry.is_dir = false;
        current_entry.size = 0;

        return;
    }

    if (selected_index >= dir_count)
    {
        selected_index = 0;
    }

    if (!sdcard_read_entry_by_index(current_path, selected_index, &current_entry))
    {
        strncpy(current_entry.name, sdcard_last_error(), sizeof(current_entry.name) - 1);
        current_entry.name[sizeof(current_entry.name) - 1] = '\0';
        current_entry.is_dir = false;
        current_entry.size = 0;
    }
}

static void browser_build_dir_index(void)
{
    if (sd_ok)
    {
        dir_count = sdcard_count_entries(current_path, MAX_DIR_ENTRIES);
        selected_index = 0;
    }
    else
    {
        dir_count = 0;
        selected_index = 0;
    }

    browser_load_current_entry();
}

static void browser_refresh_sd(void)
{
    show_message("SD INIT", "PLEASE WAIT", 300);

    sd_ok = sdcard_init();

    browser_build_dir_index();

    lcd_clear();
}

static void browser_go_root(void)
{
    strncpy(current_path, "/", sizeof(current_path) - 1);
    current_path[sizeof(current_path) - 1] = '\0';

    selected_index = 0;

    browser_build_dir_index();

    set_status_message("ROOT");
}

static void browser_go_parent(void)
{
    if (browser_is_root())
    {
        set_status_message("ROOT");
        return;
    }

    char *last_slash = strrchr(current_path, '/');

    if (last_slash == NULL)
    {
        browser_go_root();
        return;
    }

    if (last_slash == current_path)
    {
        current_path[1] = '\0';
    }
    else
    {
        *last_slash = '\0';
    }

    selected_index = 0;

    browser_build_dir_index();

    set_status_message("BACK");
}

static bool browser_enter_directory(const char *name)
{
    if (name == NULL)
    {
        return false;
    }

    if (strlen(name) == 0)
    {
        return false;
    }

    char new_path[BROWSER_PATH_MAX];

    if (browser_is_root())
    {
        snprintf(new_path, sizeof(new_path), "/%s", name);
    }
    else
    {
        snprintf(new_path, sizeof(new_path), "%s/%s", current_path, name);
    }

    if (strlen(new_path) >= sizeof(current_path))
    {
        set_status_message("PATH TOO LONG");
        return false;
    }

    strncpy(current_path, new_path, sizeof(current_path) - 1);
    current_path[sizeof(current_path) - 1] = '\0';

    selected_index = 0;

    browser_build_dir_index();

    set_status_message("ENTER DIR");

    return true;
}

static void browser_move_up(void)
{
    if (!sd_ok || dir_count == 0)
    {
        return;
    }

    if (selected_index == 0)
    {
        selected_index = dir_count - 1;
    }
    else
    {
        selected_index--;
    }

    browser_load_current_entry();
}

static void browser_move_down(void)
{
    if (!sd_ok || dir_count == 0)
    {
        return;
    }

    selected_index++;

    if (selected_index >= dir_count)
    {
        selected_index = 0;
    }

    browser_load_current_entry();
}

static void browser_select_current(void)
{
    if (!sd_ok)
    {
        browser_refresh_sd();
        return;
    }

    if (dir_count == 0)
    {
        set_status_message("EMPTY DIR");
        return;
    }

    if (current_entry.is_dir)
    {
        browser_enter_directory(current_entry.name);
    }
    else
    {
        set_status_message("FILE SELECT");
    }
}

static void browser_make_path_label(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0)
    {
        return;
    }

    if (browser_is_root())
    {
        snprintf(out, out_size, "/");
        return;
    }

    const char *last_slash = strrchr(current_path, '/');

    if (last_slash == NULL)
    {
        snprintf(out, out_size, "%s", current_path);
        return;
    }

    if (*(last_slash + 1) == '\0')
    {
        snprintf(out, out_size, "/");
        return;
    }

    snprintf(out, out_size, "%s", last_slash + 1);
}

void browser_init(bool initial_sd_ok)
{
    sd_ok = initial_sd_ok;

    strncpy(current_path, "/", sizeof(current_path) - 1);
    current_path[sizeof(current_path) - 1] = '\0';

    dir_count = 0;
    selected_index = 0;

    memset(&current_entry, 0, sizeof(current_entry));

    status_message[0] = '\0';
    status_message_until_ms = 0;

    browser_build_dir_index();
}

void browser_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
            browser_move_up();
            break;

        case BUTTON_EVENT_DOWN_PRESS:
        case BUTTON_EVENT_DOWN_REPEAT:
            browser_move_down();
            break;

        case BUTTON_EVENT_LEFT_SHORT:
            browser_go_parent();
            break;

        case BUTTON_EVENT_LEFT_LONG:
            browser_go_root();
            break;

        case BUTTON_EVENT_RIGHT_SHORT:
            browser_refresh_sd();
            break;

        case BUTTON_EVENT_SELECT_SHORT:
            browser_select_current();
            break;

        case BUTTON_EVENT_SELECT_LONG:
            set_status_message("MENU TODO");
            break;

        default:
            break;
    }
}

void browser_render(void)
{
    char line0[17];
    char line1[17];

    uint32_t now = millis();

    if (now < status_message_until_ms)
    {
        lcd_set_cursor(0, 0);
        lcd_print(status_message);

        if (sd_ok)
        {
            lcd_set_cursor(0, 1);
            lcd_print("                ");
        }
        else
        {
            snprintf(
                line1,
                sizeof(line1),
                "E:%02X D:%02X      ",
                sdcard_last_error_code(),
                sdcard_last_error_data()
            );

            lcd_set_cursor(0, 1);
            lcd_print(line1);
        }

        return;
    }

    if (!sd_ok)
    {
        snprintf(
            line0,
            sizeof(line0),
            "SD E:%02X D:%02X",
            sdcard_last_error_code(),
            sdcard_last_error_data()
        );

        snprintf(
            line1,
            sizeof(line1),
            "%-16s",
            current_entry.name
        );
    }
    else
    {
        if (dir_count == 0)
        {
            char path_label[9];

            browser_make_path_label(path_label, sizeof(path_label));

            snprintf(
                line0,
                sizeof(line0),
                "%-8s  0/0",
                path_label
            );

            snprintf(
                line1,
                sizeof(line1),
                "EMPTY           "
            );
        }
        else
        {
            char path_label[9];

            browser_make_path_label(path_label, sizeof(path_label));

            snprintf(
                line0,
                sizeof(line0),
                "%-8s%3u/%3u",
                path_label,
                (unsigned int)(selected_index + 1),
                (unsigned int)dir_count
            );

            snprintf(
                line1,
                sizeof(line1),
                "%c:%-14s",
                current_entry.is_dir ? 'D' : 'F',
                current_entry.name
            );
        }
    }

    lcd_set_cursor(0, 0);
    lcd_print(line0);

    lcd_set_cursor(0, 1);
    lcd_print(line1);
}

const char* browser_get_selected_name(void)
{
    return current_entry.name;
}

bool browser_selected_is_directory(void)
{
    return current_entry.is_dir;
}