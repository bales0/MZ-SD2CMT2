#include "record_screen.h"

#include <Arduino.h>

#include "../drivers/lcd.h"
#include "../drivers/flash_text.h"
#include "../record/record_engine.h"

static const char text_rec_fallback[] PROGMEM = "REC";
static const char text_wav_rate_22k[] PROGMEM = "22k";
static const char text_wav_rate_44k[] PROGMEM = "44k";
static const char text_wait_signal[] PROGMEM = "WAIT SIGNAL B%03u";
static const char text_wait_motor[] PROGMEM = "WAIT MOTOR B%03u";
static const char text_recording[] PROGMEM = "REC %02lu:%02lu B%03u";
/* Exactly 16 columns: PAU mm:ss Bxxx <M|U>. */
static const char text_paused[] PROGMEM = "PAU %02lu:%02lu B%03u %c";
static const char text_saving[] PROGMEM = "SAVING %-9.9s";
static const char text_please_wait[] PROGMEM = "PLEASE WAIT";
static const char text_saved[] PROGMEM = "SAVED %-10.10s";
static const char text_left_back[] PROGMEM = "LEFT = BACK";
static const char text_file_deleted[] PROGMEM = "FILE DELETED";
static const char text_rec_cancelled[] PROGMEM = "REC CANCELLED";
static const char text_rec_error[] PROGMEM = "REC ERROR";

static void line(uint8_t row, const char *text)
{
    char output[17];
    uint8_t length = 0U;

    if (text != NULL)
    {
        while ((length < 16U) && (text[length] != '\0'))
        {
            output[length] = text[length];
            ++length;
        }
    }
    while (length < 16U) output[length++] = ' ';
    output[16] = '\0';
    lcd_set_cursor(0, row);
    lcd_print(output);
}

static void copy_filename_line(char *destination, const char *filename)
{
    if ((filename == NULL) || (filename[0] == '\0'))
    {
        flash_text_copy(destination, 17U, text_rec_fallback);
        return;
    }
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        destination[i] = filename[i];
        if (filename[i] == '\0') return;
    }
    destination[16] = '\0';
}

static PGM_P wav_rate_label_P(uint32_t sample_rate)
{
    return (sample_rate >= 40000UL) ? text_wav_rate_44k : text_wav_rate_22k;
}

/* RECxxxx.WAV is exactly 11 characters. The compact flash suffix is right
   aligned to the final LCD columns: REC0001.WAV  44k. */
static void copy_wav_filename_line(char *destination, const char *filename,
                                   uint32_t sample_rate)
{
    char rate[4];
    uint8_t index = 0U;
    uint8_t rate_length = 0U;
    uint8_t rate_start;

    flash_text_copy(rate, sizeof(rate), wav_rate_label_P(sample_rate));
    while ((rate_length < (uint8_t)(sizeof(rate) - 1U)) &&
           (rate[rate_length] != '\0'))
    {
        ++rate_length;
    }
    rate_start = (uint8_t)(16U - rate_length);

    if ((filename == NULL) || (filename[0] == '\0'))
    {
        flash_text_copy(destination, 17U, text_rec_fallback);
        while ((destination[index] != '\0') && (index < rate_start))
        {
            ++index;
        }
    }
    else
    {
        while ((index < rate_start) && (filename[index] != '\0'))
        {
            destination[index] = filename[index];
            ++index;
        }
    }

    while (index < rate_start)
    {
        destination[index++] = ' ';
    }
    for (uint8_t i = 0U; i < rate_length; ++i)
    {
        destination[index++] = rate[i];
    }
    destination[16] = '\0';
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
    char filename_copy[17];
    uint32_t seconds = record_engine_get_elapsed_seconds();
    char pause_indicator = record_engine_get_pause_indicator();

    /* The actual RECxxxx filename deliberately remains in row 0 after MOTOR LOW.
       WAV adds the active configured sample-rate suffix without consuming RAM. */
    if (record_engine_get_format() == FILE_FORMAT_WAV)
    {
        copy_wav_filename_line(line0, filename, record_engine_get_wav_sample_rate());
    }
    else
    {
        copy_filename_line(line0, filename);
    }
    copy_filename_line(filename_copy, filename);

    switch (state)
    {
        case RECORD_ENGINE_ARMED:
            flash_text_snprintf(line1, sizeof(line1),
                                (record_engine_get_control_mode() == RECORD_CONTROL_AUTO) ?
                                text_wait_signal : text_wait_motor,
                                (unsigned int)record_engine_get_buffer_fill_percent());
            break;
        case RECORD_ENGINE_RECORDING:
            flash_text_snprintf(line1, sizeof(line1), text_recording,
                                (unsigned long)((seconds / 60UL) % 100UL),
                                (unsigned long)(seconds % 60UL),
                                (unsigned int)record_engine_get_buffer_fill_percent());
            break;
        case RECORD_ENGINE_PAUSED:
            flash_text_snprintf(line1, sizeof(line1), text_paused,
                                (unsigned long)((seconds / 60UL) % 100UL),
                                (unsigned long)(seconds % 60UL),
                                (unsigned int)record_engine_get_buffer_fill_percent(),
                                (pause_indicator == '\0') ? '?' : pause_indicator);
            break;
        case RECORD_ENGINE_FINALIZING:
            flash_text_snprintf(line0, sizeof(line0), text_saving, filename_copy);
            flash_text_copy(line1, sizeof(line1), text_please_wait);
            break;
        case RECORD_ENGINE_FINISHED:
            flash_text_snprintf(line0, sizeof(line0), text_saved, filename_copy);
            flash_text_copy(line1, sizeof(line1), text_left_back);
            break;
        case RECORD_ENGINE_CANCELLED:
            flash_text_copy(line0, sizeof(line0), record_engine_cancelled_file_removed() ?
                            text_file_deleted : text_rec_cancelled);
            flash_text_copy(line1, sizeof(line1), text_left_back);
            break;
        case RECORD_ENGINE_ERROR:
            flash_text_copy(line0, sizeof(line0), text_rec_error);
            copy_filename_line(line1, record_engine_get_error_text());
            break;
        case RECORD_ENGINE_STOPPED:
        default:
            flash_text_copy(line1, sizeof(line1), text_left_back);
            break;
    }

    line(0U, line0);
    line(1U, line1);
}
