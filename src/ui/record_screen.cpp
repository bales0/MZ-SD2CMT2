#include "record_screen.h"

#include <Arduino.h>
#include <stdio.h>

#include "../drivers/lcd.h"
#include "../record/wav_record_engine.h"

static void record_lcd_print_fixed(uint8_t row, const char *text)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16s", text != NULL ? text : "");
    lcd_set_cursor(0, row);
    lcd_print(line);
}

record_screen_action_t record_screen_handle_event(button_event_t event)
{
    switch (event)
    {
        case BUTTON_EVENT_SELECT_SHORT:
        case BUTTON_EVENT_LEFT_SHORT:
        case BUTTON_EVENT_LEFT_LONG:
            return RECORD_SCREEN_ACTION_STOP_OR_BACK;

        default:
            break;
    }

    return RECORD_SCREEN_ACTION_NONE;
}

void record_screen_render(void)
{
    wav_record_engine_state_t state = wav_record_engine_get_state();
    const char *filename = wav_record_engine_get_filename();
    char line0[17];
    char line1[17];

    if ((filename == NULL) || (filename[0] == '\0'))
    {
        snprintf(line0, sizeof(line0), "RECORD");
    }
    else
    {
        snprintf(line0, sizeof(line0), "REC %-12s", filename);
    }

    switch (state)
    {
        case WAV_RECORD_ENGINE_RECORDING:
        {
            uint32_t samples = wav_record_engine_get_captured_samples();
            uint32_t rate = wav_record_engine_get_sample_rate();
            uint32_t total_seconds = (rate == 0UL) ? 0UL : (samples / rate);
            uint32_t minutes = total_seconds / 60UL;
            uint32_t seconds = total_seconds % 60UL;

            snprintf(line1, sizeof(line1), "%5lu %02lu:%02lu H%02u",
                     (unsigned long)rate,
                     (unsigned long)(minutes % 100UL),
                     (unsigned long)seconds,
                     (unsigned int)wav_record_engine_get_buffer_headroom_percent());
            break;
        }

        case WAV_RECORD_ENGINE_FINALIZING:
            snprintf(line0, sizeof(line0), "SAVING %-9s",
                     filename != NULL ? filename : "");
            snprintf(line1, sizeof(line1), "PLEASE WAIT");
            break;

        case WAV_RECORD_ENGINE_FINISHED:
            snprintf(line0, sizeof(line0), "SAVED %-10s",
                     filename != NULL ? filename : "");
            snprintf(line1, sizeof(line1), "SELECT/LEFT BACK");
            break;

        case WAV_RECORD_ENGINE_ERROR:
            snprintf(line0, sizeof(line0), "REC ERROR");
            snprintf(line1, sizeof(line1), "%-16s",
                     wav_record_engine_get_error_text());
            break;

        case WAV_RECORD_ENGINE_STOPPED:
        default:
            snprintf(line1, sizeof(line1), "NOT RECORDING");
            break;
    }

    record_lcd_print_fixed(0, line0);
    record_lcd_print_fixed(1, line1);
}
