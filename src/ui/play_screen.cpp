#include <Arduino.h>

#include "play_screen.h"
#include "../drivers/lcd.h"
#include "../drivers/flash_text.h"
#include "../formats/file_format.h"

static const char text_play[] PROGMEM = "PLAY";
static const char text_no_session[] PROGMEM = "NO SESSION";
static const char text_ply[] PROGMEM = "PLY";
static const char text_pau[] PROGMEM = "PAU";
static const char text_err[] PROGMEM = "ERR";
static const char text_rdy[] PROGMEM = "RDY";
static const char text_error[] PROGMEM = "ERR %.12s";
static const char text_unknown[] PROGMEM = "UNKNOWN";
static const char text_unsupported[] PROGMEM = "UNSUPPORTED";
static const char text_wait_time[] PROGMEM = "WAIT %02u:%02u/%02u:%02u";
static const char text_play_time[] PROGMEM = "%s %02u:%02u/%02u:%02u";
static const char text_wait_percent[] PROGMEM = "WAIT %03u%%";
static const char text_play_percent[] PROGMEM = "%s %03u%%";

static void lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];
    uint8_t length = 0U;
    if (text != NULL)
    {
        while ((length < 16U) && (text[length] != '\0'))
        {
            line[length] = text[length];
            ++length;
        }
    }
    while (length < 16U) line[length++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0U, row);
    lcd_print(line);
}

static void lcd_print_fixed_P(uint8_t row, PGM_P text)
{
    char line[17];
    flash_text_copy(line, sizeof(line), text);
    lcd_print_fixed(row, line);
}

static PGM_P state_to_label_P(play_controller_state_t state)
{
    switch (state)
    {
        case PLAY_CONTROLLER_STATE_PLAYING: return text_ply;
        case PLAY_CONTROLLER_STATE_PAUSED: return text_pau;
        case PLAY_CONTROLLER_STATE_ERROR: return text_err;
        default: return text_rdy;
    }
}

/* The 16-column LCD accommodates MM:SS/MM:SS. Clamp only pathological files
   longer than 99:59; normal CMT recordings remain fully represented. */
static void play_time_parts(uint32_t milliseconds,
                            uint8_t *minutes,
                            uint8_t *seconds)
{
    uint32_t total_seconds = milliseconds / 1000UL;
    if (total_seconds > 5999UL) total_seconds = 5999UL;
    *minutes = (uint8_t)(total_seconds / 60UL);
    *seconds = (uint8_t)(total_seconds % 60UL);
}

static char play_pause_indicator(const play_controller_view_t *view)
{
    if ((view == NULL) || (view->state != PLAY_CONTROLLER_STATE_PAUSED))
    {
        return '\0';
    }
    if (view->paused_by_user) return 'U';
    if (view->paused_by_motor) return 'M';
    return '\0';
}

/* Put an M/U reason in the final LCD column without shortening the normal
   time or percentage display. */
static void append_pause_indicator(char *line, char indicator)
{
    uint8_t length = 0U;

    if ((line == NULL) || (indicator == '\0')) return;

    while ((length < 16U) && (line[length] != '\0')) ++length;
    while (length < 16U) line[length++] = ' ';
    line[15] = indicator;
    line[16] = '\0';
}

play_screen_action_t play_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT_SHORT: return PLAY_SCREEN_ACTION_TOGGLE_PLAY;
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG: return PLAY_SCREEN_ACTION_BACK;
        default: return PLAY_SCREEN_ACTION_NONE;
    }
}

void play_screen_render(const play_controller_view_t *view)
{
    char line1[17];
    char label[4];
    char error_fallback[17];
    uint8_t elapsed_minutes;
    uint8_t elapsed_seconds;
    uint8_t total_minutes;
    uint8_t total_seconds;
    char pause_indicator;

    if (view == NULL)
    {
        lcd_print_fixed_P(0U, text_play);
        lcd_print_fixed_P(1U, text_no_session);
        return;
    }

    lcd_print_fixed(0U, view->filename);
    if (view->state == PLAY_CONTROLLER_STATE_ERROR)
    {
        const char *error_text = view->error_text;
        if (error_text == NULL)
        {
            flash_text_copy(error_fallback, sizeof(error_fallback), text_unknown);
            error_text = error_fallback;
        }
        flash_text_snprintf(line1, sizeof(line1), text_error, error_text);
    }
    else if (view->format == FILE_FORMAT_UNKNOWN)
    {
        flash_text_copy(line1, sizeof(line1), text_unsupported);
    }
    else
    {
        if (view->progress_is_percent)
        {
            if (view->waiting_for_motor)
            {
                flash_text_snprintf(line1, sizeof(line1), text_wait_percent,
                                    (unsigned int)view->progress_percent);
            }
            else
            {
                flash_text_copy(label, sizeof(label), state_to_label_P(view->state));
                flash_text_snprintf(line1, sizeof(line1), text_play_percent, label,
                                    (unsigned int)view->progress_percent);
            }
        }
        else
        {
            play_time_parts(view->elapsed_ms, &elapsed_minutes, &elapsed_seconds);
            play_time_parts(view->total_duration_ms, &total_minutes, &total_seconds);

            if (view->waiting_for_motor)
            {
                flash_text_snprintf(line1, sizeof(line1), text_wait_time,
                                    (unsigned int)elapsed_minutes,
                                    (unsigned int)elapsed_seconds,
                                    (unsigned int)total_minutes,
                                    (unsigned int)total_seconds);
            }
            else
            {
                flash_text_copy(label, sizeof(label), state_to_label_P(view->state));
                flash_text_snprintf(line1, sizeof(line1), text_play_time, label,
                                    (unsigned int)elapsed_minutes,
                                    (unsigned int)elapsed_seconds,
                                    (unsigned int)total_minutes,
                                    (unsigned int)total_seconds);
            }
        }
    }

    pause_indicator = play_pause_indicator(view);
    append_pause_indicator(line1, pause_indicator);
    lcd_print_fixed(1U, line1);
}
