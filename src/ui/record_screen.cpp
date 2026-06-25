#include "record_screen.h"

#include <Arduino.h>
#include <stdio.h>

#include "../drivers/lcd.h"
#include "../record/record_engine.h"
#include "record_menu.h"

static void line(uint8_t row, const char *text)
{
    char output[17];
    snprintf(output, sizeof(output), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(output);
}

record_screen_action_t record_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT_SHORT:
            return RECORD_SCREEN_ACTION_TOGGLE_PAUSE;
        case BUTTON_EVENT_LEFT_SHORT:
            return RECORD_SCREEN_ACTION_STOP_SAVE;
        case BUTTON_EVENT_LEFT_LONG:
            return RECORD_SCREEN_ACTION_CANCEL_BACK;
        /* RIGHT short/long intentionally has no function while recording. */
        default:
            return RECORD_SCREEN_ACTION_NONE;
    }
}

void record_screen_render(void)
{
    record_engine_state_t state = record_engine_get_state();
    const char *filename = record_engine_get_filename();
    char line0[17];
    char line1[17];
    const char *mode = record_control_mode_label(record_engine_get_display_control_mode());
    uint32_t seconds = record_engine_get_elapsed_seconds();

    snprintf(line0, sizeof(line0), "REC %-12.12s", filename != NULL ? filename : "");

    switch (state)
    {
        case RECORD_ENGINE_ARMED:
            /* Keep the normal REC <future filename> identity on row 0.
               Row 1 alone carries the armed/waiting transport state. */
            if (record_engine_get_control_mode() == RECORD_CONTROL_AUTO)
            {
                snprintf(line1, sizeof(line1), "WAIT SIGNAL B%03u",
                         (unsigned int)record_engine_get_buffer_fill_percent());
            }
            else
            {
                snprintf(line1, sizeof(line1), "WAIT MOTOR B%03u",
                         (unsigned int)record_engine_get_buffer_fill_percent());
            }
            break;
        case RECORD_ENGINE_RECORDING:
            snprintf(line1, sizeof(line1), "REC %02lu:%02lu B%03u",
                     (unsigned long)((seconds / 60UL) % 100UL),
                     (unsigned long)(seconds % 60UL),
                     (unsigned int)record_engine_get_buffer_fill_percent());
            break;
        case RECORD_ENGINE_PAUSED:
            snprintf(line1, sizeof(line1), "PAU %02lu:%02lu %-4s",
                     (unsigned long)((seconds / 60UL) % 100UL),
                     (unsigned long)(seconds % 60UL), mode);
            break;
        case RECORD_ENGINE_FINALIZING:
            snprintf(line0, sizeof(line0), "SAVING %-9.9s", filename != NULL ? filename : "");
            snprintf(line1, sizeof(line1), "PLEASE WAIT");
            break;
        case RECORD_ENGINE_FINISHED:
            snprintf(line0, sizeof(line0), "SAVED %-10.10s", filename != NULL ? filename : "");
            snprintf(line1, sizeof(line1), "LEFT = BACK");
            break;
        case RECORD_ENGINE_CANCELLED:
            if (record_engine_cancelled_file_removed())
            {
                snprintf(line0, sizeof(line0), "FILE DELETED");
            }
            else
            {
                snprintf(line0, sizeof(line0), "REC CANCELLED");
            }
            snprintf(line1, sizeof(line1), "LEFT = BACK");
            break;
        case RECORD_ENGINE_ERROR:
            snprintf(line0, sizeof(line0), "REC ERROR");
            snprintf(line1, sizeof(line1), "%.16s", record_engine_get_error_text());
            break;
        case RECORD_ENGINE_STOPPED:
        default:
            snprintf(line1, sizeof(line1), "LEFT = BACK");
            break;
    }

    line(0, line0);
    line(1, line1);
}
