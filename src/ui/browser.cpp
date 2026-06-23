#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "browser.h"
#include "../drivers/lcd.h"
#include "../drivers/sdcard.h"

#define MAX_DIR_ENTRIES 999
#define BROWSER_PATH_MAX 96
#define BROWSER_NAME_MAX 64
#define BROWSER_FULL_PATH_MAX 160
#define BROWSER_PARENT_STACK_MAX 16
#define MESSAGE_HOLD_MS 1200

static bool sd_ok = false;
static uint16_t dir_count = 0;
static uint16_t selected_index = 0;
static char current_path[BROWSER_PATH_MAX] = "/";
static sdcard_entry_t current_entry;
static char selected_full_path[BROWSER_FULL_PATH_MAX];
static bool saved_position_valid = false;
static char saved_path[BROWSER_PATH_MAX] = "/";
static char saved_name[BROWSER_NAME_MAX];
static bool saved_is_dir = false;
static uint16_t saved_index = 0;
static uint8_t parent_stack_depth = 0;
static char parent_stack_name[BROWSER_PARENT_STACK_MAX][BROWSER_NAME_MAX];
static bool parent_stack_is_dir[BROWSER_PARENT_STACK_MAX];
static uint16_t parent_stack_index[BROWSER_PARENT_STACK_MAX];
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

static void browser_update_selected_full_path(void)
{
    if (current_entry.name[0] == '\0')
    {
        selected_full_path[0] = '\0';
        return;
    }
    if (browser_is_root())
    {
        snprintf(selected_full_path, sizeof(selected_full_path), "/%s", current_entry.name);
    }
    else
    {
        snprintf(selected_full_path, sizeof(selected_full_path), "%s/%s", current_path, current_entry.name);
    }
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
        browser_update_selected_full_path();
        return;
    }
    if (dir_count == 0)
    {
        strncpy(current_entry.name, "EMPTY", sizeof(current_entry.name) - 1);
        current_entry.name[sizeof(current_entry.name) - 1] = '\0';
        current_entry.is_dir = false;
        current_entry.size = 0;
        browser_update_selected_full_path();
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
    browser_update_selected_full_path();
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

static bool browser_restore_entry_by_identity(const char *name, bool is_dir, uint16_t preferred_index)
{
    sdcard_entry_t entry;
    if ((name == NULL) || !sd_ok || (dir_count == 0))
    {
        return false;
    }
    if (preferred_index < dir_count)
    {
        memset(&entry, 0, sizeof(entry));
        if (sdcard_read_entry_by_index(current_path, preferred_index, &entry))
        {
            if ((entry.is_dir == is_dir) && (strcmp(entry.name, name) == 0))
            {
                selected_index = preferred_index;
                browser_load_current_entry();
                return true;
            }
        }
    }
    for (uint16_t index = 0; index < dir_count; index++)
    {
        memset(&entry, 0, sizeof(entry));
        if (!sdcard_read_entry_by_index(current_path, index, &entry))
        {
            continue;
        }
        if ((entry.is_dir == is_dir) && (strcmp(entry.name, name) == 0))
        {
            selected_index = index;
            browser_load_current_entry();
            return true;
        }
    }
    return false;
}

static bool browser_find_saved_position(void)
{
    if (!saved_position_valid)
    {
        return false;
    }
    if (strcmp(current_path, saved_path) != 0)
    {
        return false;
    }
    return browser_restore_entry_by_identity(saved_name, saved_is_dir, saved_index);
}

static void browser_push_parent_position(void)
{
    if (parent_stack_depth >= BROWSER_PARENT_STACK_MAX)
    {
        return;
    }
    strncpy(parent_stack_name[parent_stack_depth], current_entry.name, sizeof(parent_stack_name[parent_stack_depth]) - 1);
    parent_stack_name[parent_stack_depth][sizeof(parent_stack_name[parent_stack_depth]) - 1] = '\0';
    parent_stack_is_dir[parent_stack_depth] = current_entry.is_dir;
    parent_stack_index[parent_stack_depth] = selected_index;
    parent_stack_depth++;
}

static bool browser_pop_parent_position_and_restore(void)
{
    if (parent_stack_depth == 0)
    {
        return false;
    }
    parent_stack_depth--;
    return browser_restore_entry_by_identity(parent_stack_name[parent_stack_depth], parent_stack_is_dir[parent_stack_depth], parent_stack_index[parent_stack_depth]);
}

static void browser_refresh_sd(void)
{
    browser_save_position();
    show_message("SD INIT", "PLEASE WAIT", 300);
    sd_ok = sdcard_init();
    browser_build_dir_index();
    browser_find_saved_position();
    lcd_clear();
}

static void browser_go_root(void)
{
    strncpy(current_path, "/", sizeof(current_path) - 1);
    current_path[sizeof(current_path) - 1] = '\0';
    selected_index = 0;
    saved_position_valid = false;
    parent_stack_depth = 0;
    browser_build_dir_index();
}

static void browser_go_parent(void)
{
    if (browser_is_root())
    {
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
    saved_position_valid = false;
    browser_build_dir_index();
    browser_pop_parent_position_and_restore();
}

static bool browser_enter_directory(const char *name)
{
    if ((name == NULL) || (strlen(name) == 0))
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
    browser_push_parent_position();
    strncpy(current_path, new_path, sizeof(current_path) - 1);
    current_path[sizeof(current_path) - 1] = '\0';
    selected_index = 0;
    saved_position_valid = false;
    browser_build_dir_index();
    return true;
}

static void browser_move_up(void)
{
    if (!sd_ok || dir_count == 0)
    {
        return;
    }
    selected_index = (selected_index == 0) ? (dir_count - 1) : (selected_index - 1);
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

static browser_action_t browser_select_current(void)
{
    if (!sd_ok)
    {
        browser_refresh_sd();
        return BROWSER_ACTION_NONE;
    }
    if (dir_count == 0)
    {
        set_status_message("EMPTY DIR");
        return BROWSER_ACTION_NONE;
    }
    if (current_entry.is_dir)
    {
        browser_enter_directory(current_entry.name);
        return BROWSER_ACTION_NONE;
    }
    set_status_message("FILE SELECT");
    return BROWSER_ACTION_FILE_SELECTED;
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
    if ((last_slash == NULL) || (*(last_slash + 1) == '\0'))
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
    selected_full_path[0] = '\0';
    saved_position_valid = false;
    saved_path[0] = '\0';
    saved_name[0] = '\0';
    saved_is_dir = false;
    saved_index = 0;
    parent_stack_depth = 0;
    status_message[0] = '\0';
    status_message_until_ms = 0;
    browser_build_dir_index();
}

browser_action_t browser_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_UP_PRESS:
        case BUTTON_EVENT_UP_REPEAT:
            browser_move_up();
            return BROWSER_ACTION_NONE;
        case BUTTON_EVENT_DOWN_PRESS:
        case BUTTON_EVENT_DOWN_REPEAT:
            browser_move_down();
            return BROWSER_ACTION_NONE;
        case BUTTON_EVENT_LEFT_SHORT:
            browser_go_parent();
            return BROWSER_ACTION_NONE;
        case BUTTON_EVENT_LEFT_LONG:
            browser_go_root();
            return BROWSER_ACTION_NONE;
        case BUTTON_EVENT_RIGHT_SHORT:
            browser_refresh_sd();
            return BROWSER_ACTION_NONE;
        case BUTTON_EVENT_SELECT_SHORT:
            return browser_select_current();
        case BUTTON_EVENT_SELECT_LONG:
            return BROWSER_ACTION_MENU_REQUESTED;
        default:
            break;
    }
    return BROWSER_ACTION_NONE;
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
            snprintf(line1, sizeof(line1), "E:%02X D:%02X      ", sdcard_last_error_code(), sdcard_last_error_data());
            lcd_set_cursor(0, 1);
            lcd_print(line1);
        }
        return;
    }

    if (!sd_ok)
    {
        snprintf(line0, sizeof(line0), "SD E:%02X D:%02X", sdcard_last_error_code(), sdcard_last_error_data());
        snprintf(line1, sizeof(line1), "%-16s", current_entry.name);
    }
    else if (dir_count == 0)
    {
        char path_label[9];
        browser_make_path_label(path_label, sizeof(path_label));
        snprintf(line0, sizeof(line0), "%-8s  0/0", path_label);
        snprintf(line1, sizeof(line1), "EMPTY           ");
    }
    else
    {
        char path_label[9];
        browser_make_path_label(path_label, sizeof(path_label));
        snprintf(line0, sizeof(line0), "%-8s%3u/%3u", path_label, (unsigned int)(selected_index + 1), (unsigned int)dir_count);
        snprintf(line1, sizeof(line1), "%c:%-14s", current_entry.is_dir ? 'D' : 'F', current_entry.name);
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

const char* browser_get_selected_full_path(void)
{
    browser_update_selected_full_path();
    return selected_full_path;
}

bool browser_selected_is_directory(void)
{
    return current_entry.is_dir;
}

void browser_save_position(void)
{
    strncpy(saved_path, current_path, sizeof(saved_path) - 1);
    saved_path[sizeof(saved_path) - 1] = '\0';
    strncpy(saved_name, current_entry.name, sizeof(saved_name) - 1);
    saved_name[sizeof(saved_name) - 1] = '\0';
    saved_is_dir = current_entry.is_dir;
    saved_index = selected_index;
    saved_position_valid = true;
}

void browser_restore_saved_position(void)
{
    if (!saved_position_valid)
    {
        return;
    }
    if (!browser_find_saved_position())
    {
        browser_load_current_entry();
    }
}
