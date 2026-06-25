#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "browser.h"
#include "../drivers/lcd.h"
#include "../drivers/sdcard.h"
#include "../streams/cmt_mode_scratch.h"

#define MAX_DIR_ENTRIES 999
#define BROWSER_PATH_MAX 96
#define BROWSER_NAME_MAX 32
#define BROWSER_FULL_PATH_MAX 160
#define BROWSER_PARENT_STACK_MAX 16
#define MESSAGE_HOLD_MS 1200
#define LCD_COLUMNS 16U
#define NAME_SCROLL_START_MS 900U
#define NAME_SCROLL_STEP_MS 260U
#define NAME_SCROLL_END_PAUSE_MS 900U
#define BROWSER_ROOT_RETRY_DELAY_MS 80U
#define BROWSER_ROOT_RETRY_COUNT 3U

static bool sd_ok = false;
static uint16_t dir_count = 0;
/* Index in alphabetical browser order, not physical FAT directory order. */
static uint16_t selected_index = 0;
static char current_path[BROWSER_PATH_MAX] = "/";
static sdcard_entry_t current_entry;
static char selected_full_path[BROWSER_FULL_PATH_MAX];
static bool saved_position_valid = false;
static char saved_path[BROWSER_PATH_MAX] = "/";
static char saved_name[BROWSER_NAME_MAX];
static bool saved_is_dir = false;
static uint8_t parent_stack_depth = 0;
static bool parent_stack_is_dir[BROWSER_PARENT_STACK_MAX];
static char status_message[17];
static uint32_t status_message_until_ms = 0;
static bool right_locked_until_release = false;

/* Horizontal position of the selected filename on the 16-character LCD. */
static uint8_t name_scroll_offset = 0U;
static uint32_t name_scroll_next_ms = 0U;

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

static void browser_reset_name_scroll(void)
{
    name_scroll_offset = 0U;
    name_scroll_next_ms = millis() + NAME_SCROLL_START_MS;
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

static void browser_set_current_entry_message(const char *message)
{
    memset(&current_entry, 0, sizeof(current_entry));
    strncpy(current_entry.name, message, sizeof(current_entry.name) - 1U);
    current_entry.name[sizeof(current_entry.name) - 1U] = '\0';
    current_entry.is_dir = false;
    current_entry.size = 0;
    browser_reset_name_scroll();
    browser_update_selected_full_path();
}

typedef enum
{
    BROWSER_DIRECTORY_LOADED = 0,
    BROWSER_DIRECTORY_EMPTY,
    BROWSER_DIRECTORY_READ_FAILED
} browser_directory_load_result_t;

static void browser_set_entry_read_error(void)
{
    const char *error_message;

    sd_ok = sdcard_is_mounted();
    if (!sd_ok)
    {
        dir_count = 0U;
        browser_set_current_entry_message("SD CARD ERROR");
        return;
    }

    error_message = sdcard_last_error();
    if ((error_message == NULL) || (error_message[0] == '\0') ||
        (strcmp(error_message, "OK") == 0) || (strcmp(error_message, "NO ENTRY") == 0))
    {
        browser_set_current_entry_message("DIR FAIL");
        return;
    }

    browser_set_current_entry_message(error_message);
}

/*
    Root and normal directory refreshes use one physical scan that returns both
    the count and the first sorted item.  Do not count and immediately reopen
    the same directory after a card initialization: some cards intermittently
    return an empty second scan even though the first scan found entries.
*/
static browser_directory_load_result_t browser_scan_current_directory(void)
{
    sdcard_entry_t first_entry;
    uint16_t scanned_count = 0U;

    if (!sd_ok)
    {
        return BROWSER_DIRECTORY_READ_FAILED;
    }

    memset(&first_entry, 0, sizeof(first_entry));
    if (!sdcard_scan_directory_first_sorted(current_path, MAX_DIR_ENTRIES,
                                            &scanned_count, &first_entry))
    {
        sd_ok = sdcard_is_mounted();
        if (!sd_ok)
        {
            dir_count = 0U;
        }
        return BROWSER_DIRECTORY_READ_FAILED;
    }

    dir_count = scanned_count;
    selected_index = 0U;

    if (dir_count == 0U)
    {
        browser_set_current_entry_message("EMPTY");
        return BROWSER_DIRECTORY_EMPTY;
    }

    current_entry = first_entry;
    browser_reset_name_scroll();
    browser_update_selected_full_path();
    return BROWSER_DIRECTORY_LOADED;
}

static bool browser_load_first_entry(void)
{
    browser_directory_load_result_t result = browser_scan_current_directory();

    if (result == BROWSER_DIRECTORY_READ_FAILED)
    {
        browser_set_entry_read_error();
        return false;
    }

    return result == BROWSER_DIRECTORY_LOADED;
}

static bool browser_load_last_entry(void)
{
    if (!sd_ok || (dir_count == 0U))
    {
        return browser_load_first_entry();
    }
    if (!sdcard_read_last_sorted_entry(current_path, dir_count, &current_entry))
    {
        browser_set_entry_read_error();
        return false;
    }
    selected_index = (uint16_t)(dir_count - 1U);
    browser_reset_name_scroll();
    browser_update_selected_full_path();
    return true;
}

static void browser_build_dir_index(void)
{
    (void)browser_load_first_entry();
}

static void browser_reset_root_state(void)
{
    strncpy(current_path, "/", sizeof(current_path) - 1U);
    current_path[sizeof(current_path) - 1U] = '\0';
    dir_count = 0U;
    selected_index = 0U;
    memset(&current_entry, 0, sizeof(current_entry));
    selected_full_path[0] = '\0';
    saved_position_valid = false;
    strncpy(saved_path, "/", sizeof(saved_path) - 1U);
    saved_path[sizeof(saved_path) - 1U] = '\0';
    saved_name[0] = '\0';
    saved_is_dir = false;
    parent_stack_depth = 0U;
    browser_reset_name_scroll();
}

/*
    A successful card initialization always starts a new browser session at
    root. The function confirms both an empty and a failed first root scan with
    a complete SPI/SdFat retry; it never keeps an old subdirectory or exposes
    a transient empty scan as a browser item.
*/
static void browser_open_root_after_sd_init(void)
{
    uint8_t attempt;

    browser_reset_root_state();

    if (!sd_ok)
    {
        browser_set_entry_read_error();
        return;
    }

    for (attempt = 0U; attempt < BROWSER_ROOT_RETRY_COUNT; ++attempt)
    {
        browser_directory_load_result_t result = browser_scan_current_directory();

        if (result == BROWSER_DIRECTORY_LOADED)
        {
            return;
        }

        /* A valid empty root is accepted only after the bootstrap retries.
           Immediately after insertion, a transient empty openNext() must not
           make a populated card look empty. */
        if ((result == BROWSER_DIRECTORY_EMPTY) &&
            ((attempt + 1U) >= BROWSER_ROOT_RETRY_COUNT))
        {
            return;
        }

        if ((result == BROWSER_DIRECTORY_READ_FAILED) &&
            ((attempt + 1U) >= BROWSER_ROOT_RETRY_COUNT))
        {
            break;
        }

        delay(BROWSER_ROOT_RETRY_DELAY_MS);
        sd_ok = sdcard_reinitialize();
        if (!sd_ok)
        {
            break;
        }

        browser_reset_root_state();
    }

    browser_set_entry_read_error();
}

static bool browser_restore_entry_by_identity(const char *name, bool is_dir)
{
    sdcard_entry_t entry;
    uint16_t resolved_index = 0U;

    if ((name == NULL) || !sd_ok || (dir_count == 0U))
    {
        return false;
    }
    memset(&entry, 0, sizeof(entry));
    if (!sdcard_find_sorted_entry_by_identity(current_path, dir_count, name, is_dir,
                                              &resolved_index, &entry))
    {
        browser_set_entry_read_error();
        return false;
    }

    selected_index = resolved_index;
    current_entry = entry;
    browser_reset_name_scroll();
    browser_update_selected_full_path();
    return true;
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
    return browser_restore_entry_by_identity(saved_name, saved_is_dir);
}

static void browser_push_parent_position(void)
{
    if (parent_stack_depth >= BROWSER_PARENT_STACK_MAX)
    {
        return;
    }
    strncpy(cmt_mode_scratch.browser_parent_names[parent_stack_depth], current_entry.name, sizeof(cmt_mode_scratch.browser_parent_names[parent_stack_depth]) - 1);
    cmt_mode_scratch.browser_parent_names[parent_stack_depth][sizeof(cmt_mode_scratch.browser_parent_names[parent_stack_depth]) - 1] = '\0';
    parent_stack_is_dir[parent_stack_depth] = current_entry.is_dir;
    parent_stack_depth++;
}

static bool browser_pop_parent_position_and_restore(void)
{
    if (parent_stack_depth == 0U)
    {
        return false;
    }
    parent_stack_depth--;
    return browser_restore_entry_by_identity(cmt_mode_scratch.browser_parent_names[parent_stack_depth], parent_stack_is_dir[parent_stack_depth]);
}

static void browser_refresh_sd(void)
{
    show_message("SD INIT", "PLEASE WAIT", 300);

    /* A card retry deliberately discards the former path and selection.  The
       media may have been replaced, so root and its first alphabetical item
       are the only deterministic restart point. */
    sd_ok = sdcard_reinitialize();
    browser_open_root_after_sd_init();
    lcd_clear();
}

static void browser_go_root(void)
{
    browser_reset_root_state();
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
    selected_index = 0U;
    saved_position_valid = false;
    browser_build_dir_index();
    browser_pop_parent_position_and_restore();
}

static bool browser_enter_directory(const char *name)
{
    if ((name == NULL) || (strlen(name) == 0U))
    {
        return false;
    }

    char new_path[BROWSER_PATH_MAX];
    size_t name_length = strlen(name);
    size_t path_length = browser_is_root() ? 1U : strlen(current_path);
    size_t required_length = path_length + name_length;

    if (!browser_is_root())
    {
        required_length++;
    }
    if (required_length >= sizeof(new_path))
    {
        set_status_message("PATH TOO LONG");
        return false;
    }

    if (browser_is_root())
    {
        new_path[0] = '/';
        memcpy(&new_path[1], name, name_length + 1U);
    }
    else
    {
        memcpy(new_path, current_path, path_length);
        new_path[path_length] = '/';
        memcpy(&new_path[path_length + 1U], name, name_length + 1U);
    }

    browser_push_parent_position();
    strncpy(current_path, new_path, sizeof(current_path) - 1U);
    current_path[sizeof(current_path) - 1U] = '\0';
    selected_index = 0U;
    saved_position_valid = false;
    browser_build_dir_index();
    return true;
}

static bool browser_load_sorted_neighbor(bool previous)
{
    sdcard_entry_t neighbor;

    if (!sdcard_read_sorted_neighbor(current_path, dir_count, &current_entry, previous, &neighbor))
    {
        return false;
    }

    current_entry = neighbor;
    if (previous)
    {
        selected_index--;
    }
    else
    {
        selected_index++;
    }
    browser_reset_name_scroll();
    browser_update_selected_full_path();
    return true;
}

static void browser_move_up(void)
{
    if (!sd_ok || (dir_count == 0U))
    {
        return;
    }
    if (selected_index == 0U)
    {
        browser_load_last_entry();
        return;
    }
    if (!browser_load_sorted_neighbor(true))
    {
        /* A no-neighbour result is a normal end-of-sort condition. Wrap
           instead of exposing the internal "NO ENTRY" diagnostic. */
        if (sdcard_is_mounted() && (strcmp(sdcard_last_error(), "NO ENTRY") == 0))
        {
            browser_load_last_entry();
        }
        else
        {
            browser_set_entry_read_error();
        }
    }
}

static void browser_move_down(void)
{
    if (!sd_ok || (dir_count == 0U))
    {
        return;
    }
    if ((uint16_t)(selected_index + 1U) >= dir_count)
    {
        browser_load_first_entry();
        return;
    }
    if (!browser_load_sorted_neighbor(false))
    {
        /* See browser_move_up(): keep a stale count or a tie at the end of
           the sort from producing a visible "NO ENTRY" line. */
        if (sdcard_is_mounted() && (strcmp(sdcard_last_error(), "NO ENTRY") == 0))
        {
            browser_load_first_entry();
        }
        else
        {
            browser_set_entry_read_error();
        }
    }
}

static browser_action_t browser_select_current(void)
{
    if (!sd_ok)
    {
        browser_refresh_sd();
        return BROWSER_ACTION_NONE;
    }
    if (dir_count == 0U)
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

static browser_action_t browser_request_record(void)
{
    /*
        RIGHT has exactly two states:
        - SD CARD ERROR: retry/reinitialize only. A successful retry never
          starts recording on the same press.
        - SD OK: verify the mounted card once and start RECORD.
    */
    if (!sd_ok)
    {
        right_locked_until_release = true;
        browser_refresh_sd();
        return BROWSER_ACTION_NONE;
    }

    /* A card may have been removed after the browser was built. */
    if (!sdcard_init())
    {
        sd_ok = false;
        browser_build_dir_index();
        return BROWSER_ACTION_NONE;
    }

    sd_ok = true;
    return BROWSER_ACTION_RECORD_REQUESTED;
}

static void browser_make_path_label(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0U)
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
    status_message[0] = '\0';
    status_message_until_ms = 0U;
    right_locked_until_release = false;
    browser_reset_root_state();

    /* setup() already completed the initial SD initialization.  Treat it like
       every later retry: always start at root and select its first item. */
    browser_open_root_after_sd_init();
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
            if (right_locked_until_release) return BROWSER_ACTION_NONE;
            return browser_request_record();
        case BUTTON_EVENT_RIGHT_LONG:
            if (right_locked_until_release) return BROWSER_ACTION_NONE;
            if (!sd_ok)
            {
                right_locked_until_release = true;
                browser_refresh_sd();
                return BROWSER_ACTION_NONE;
            }
            if (!sdcard_init())
            {
                sd_ok = false;
                browser_build_dir_index();
                return BROWSER_ACTION_NONE;
            }
            return BROWSER_ACTION_RECORD_MENU_REQUESTED;
        case BUTTON_EVENT_SELECT_SHORT:
            return browser_select_current();
        case BUTTON_EVENT_SELECT_LONG:
            return BROWSER_ACTION_PLAY_MENU_REQUESTED;
        default:
            break;
    }
    return BROWSER_ACTION_NONE;
}

void browser_service(void)
{
    uint32_t now;
    size_t name_length;
    uint8_t maximum_offset;

    if (right_locked_until_release && (keypad_get_button() != BUTTON_RIGHT))
    {
        right_locked_until_release = false;
    }

    name_length = strlen(current_entry.name);
    if (name_length <= LCD_COLUMNS)
    {
        name_scroll_offset = 0U;
        return;
    }

    now = millis();
    if ((int32_t)(now - name_scroll_next_ms) < 0)
    {
        return;
    }

    maximum_offset = (uint8_t)(name_length - LCD_COLUMNS);
    if (name_scroll_offset < maximum_offset)
    {
        name_scroll_offset++;
        if (name_scroll_offset >= maximum_offset)
        {
            name_scroll_next_ms = now + NAME_SCROLL_END_PAUSE_MS;
        }
        else
        {
            name_scroll_next_ms = now + NAME_SCROLL_STEP_MS;
        }
    }
    else
    {
        name_scroll_offset = 0U;
        name_scroll_next_ms = now + NAME_SCROLL_START_MS;
    }
}

static void browser_format_entry_line(char *line, const sdcard_entry_t *entry)
{
    size_t name_length;
    uint8_t offset;
    uint8_t column;

    memset(line, ' ', LCD_COLUMNS);
    line[LCD_COLUMNS] = '\0';

    if ((entry == NULL) || (entry->name[0] == '\0'))
    {
        return;
    }

    name_length = strlen(entry->name);
    offset = name_scroll_offset;
    if (name_length <= LCD_COLUMNS)
    {
        offset = 0U;
    }
    else if (offset > (uint8_t)(name_length - LCD_COLUMNS))
    {
        offset = 0U;
    }

    for (column = 0U; column < LCD_COLUMNS; column++)
    {
        size_t source_index = (size_t)offset + (size_t)column;
        if (source_index >= name_length)
        {
            break;
        }
        line[column] = entry->name[source_index];
    }
}

static uint8_t browser_write_unsigned(char *line, uint8_t column, uint16_t value)
{
    char digits[5];
    uint8_t digit_count = 0U;

    do
    {
        digits[digit_count++] = (char)('0' + (value % 10U));
        value = (uint16_t)(value / 10U);
    }
    while ((value != 0U) && (digit_count < sizeof(digits)));

    while (digit_count > 0U)
    {
        line[column++] = digits[--digit_count];
    }
    return column;
}

static void browser_format_position_line(char *line, const char *path_label)
{
    char counter[8];
    uint8_t counter_length;
    uint8_t counter_column;
    uint8_t path_column = 0U;
    uint16_t display_index = (dir_count == 0U) ? 0U : (uint16_t)(selected_index + 1U);
    uint16_t display_count = dir_count;

    if (display_index > MAX_DIR_ENTRIES)
    {
        display_index = MAX_DIR_ENTRIES;
    }
    if (display_count > MAX_DIR_ENTRIES)
    {
        display_count = MAX_DIR_ENTRIES;
    }

    memset(line, ' ', LCD_COLUMNS);
    line[LCD_COLUMNS] = '\0';

    /* The counter is compact but right-aligned: "/GAMES        2/3".
       The root is shown as "/            2/3". */
    counter_length = browser_write_unsigned(counter, 0U, display_index);
    counter[counter_length++] = '/';
    counter_length = browser_write_unsigned(counter, counter_length, display_count);
    counter[counter_length] = '\0';
    counter_column = (uint8_t)(LCD_COLUMNS - counter_length);

    if ((path_label != NULL) && (path_label[0] != '\0'))
    {
        while ((*path_label != '\0') && (path_column < counter_column))
        {
            line[path_column++] = *path_label++;
        }
    }

    memcpy(&line[counter_column], counter, counter_length);
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
            lcd_set_cursor(0, 1);
            lcd_print("RIGHT=RETRY     ");
        }
        return;
    }

    if (!sd_ok)
    {
        snprintf(line0, sizeof(line0), "%-16s", "SD CARD ERROR");
        snprintf(line1, sizeof(line1), "%-16s", "RIGHT=RETRY");
    }
    else if (dir_count == 0U)
    {
        char path_label[9];
        browser_make_path_label(path_label, sizeof(path_label));
        browser_format_position_line(line0, path_label);
        memset(line1, ' ', LCD_COLUMNS);
        memcpy(line1, "EMPTY", 5U);
        line1[LCD_COLUMNS] = '\0';
    }
    else
    {
        char path_label[9];
        browser_make_path_label(path_label, sizeof(path_label));
        browser_format_position_line(line0, path_label);
        browser_format_entry_line(line1, &current_entry);
    }

    /* Every generated line has exactly 16 printable characters before NUL,
       so shorter filenames cannot leave stale glyphs on HD44780 LCDs. */
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
    strncpy(saved_path, current_path, sizeof(saved_path) - 1U);
    saved_path[sizeof(saved_path) - 1U] = '\0';
    strncpy(saved_name, current_entry.name, sizeof(saved_name) - 1U);
    saved_name[sizeof(saved_name) - 1U] = '\0';
    saved_is_dir = current_entry.is_dir;
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
        browser_load_first_entry();
    }
}

void browser_begin_record_scratch(void)
{
    /* The record engine reuses the 512B parent-name area as a second stage. */
    parent_stack_depth = 0U;
}

void browser_end_record_scratch(void)
{
    /* History bytes were reused by record mode; navigation itself remains valid. */
    parent_stack_depth = 0U;
}

const char* browser_get_current_path(void)
{
    return current_path;
}

void browser_refresh(void)
{
    /* The file was just saved/deleted on an already mounted card.
       Re-scan silently: do not show the SD INIT splash on return. */
    browser_build_dir_index();
    browser_find_saved_position();
    lcd_clear();
}
