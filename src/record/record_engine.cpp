#include "record_engine.h"

#include <Arduino.h>
#include <string.h>

#include "wav_record_engine.h"
#include "edge_record_engine.h"
#include "../drivers/mzio.h"
#include "../drivers/write_edge_monitor.h"
#include "../drivers/flash_text.h"

static record_engine_state_t engine_state = RECORD_ENGINE_STOPPED;
static record_engine_config_t engine_config = { FILE_FORMAT_WAV, 22050UL, RECORD_CONTROL_MOTOR };
static char engine_directory[96];
static char engine_error[17];
static bool capture_started = false;

/* Manual pause dominates MOTOR. A user resume clears only this lock; the
   current MOTOR level is then applied again. */
static bool paused_by_motor = false;
static bool paused_by_user = false;

static uint32_t active_started_ms = 0UL;
static uint32_t pause_started_ms = 0UL;
static uint32_t paused_total_ms = 0UL;
static bool cancelled_file_removed = false;
static uint16_t auto_last_edge_count = 0U;
static uint32_t auto_last_activity_ms = 0UL;
#define AUTO_RECORD_IDLE_STOP_MS 5000UL

static bool is_edge_format(file_format_t format)
{
    return (format == FILE_FORMAT_LEP) || (format == FILE_FORMAT_L16);
}

static void record_engine_clear_pause_reason(void)
{
    paused_by_motor = false;
    paused_by_user = false;
}

static void record_engine_set_error(const char *text)
{
    if (text == NULL)
    {
        flash_text_copy(engine_error, sizeof(engine_error), PSTR("REC ERROR"));
    }
    else
    {
        strncpy(engine_error, text, sizeof(engine_error) - 1U);
        engine_error[sizeof(engine_error) - 1U] = '\0';
    }
    engine_state = RECORD_ENGINE_ERROR;
    record_engine_clear_pause_reason();
    write_edge_monitor_stop();
    mz_sense_set(true);
}

static void record_engine_set_error_P(PGM_P text)
{
    flash_text_copy(engine_error, sizeof(engine_error), text);
    engine_state = RECORD_ENGINE_ERROR;
    record_engine_clear_pause_reason();
    write_edge_monitor_stop();
    mz_sense_set(true);
}

static bool record_engine_begin_capture(void)
{
    bool ok;

    if (capture_started)
    {
        return true;
    }

    if (engine_config.format == FILE_FORMAT_WAV)
    {
        ok = wav_record_engine_start(engine_directory, engine_config.wav_sample_rate);
    }
    else if (is_edge_format(engine_config.format))
    {
        ok = edge_record_engine_start(engine_directory, engine_config.format);
    }
    else
    {
        record_engine_set_error_P(PSTR("REC FORMAT"));
        return false;
    }

    if (!ok)
    {
        const char *error = (engine_config.format == FILE_FORMAT_WAV) ?
            wav_record_engine_get_error_text() : edge_record_engine_get_error_text();
        record_engine_set_error(error);
        return false;
    }

    capture_started = true;
    engine_state = RECORD_ENGINE_RECORDING;
    record_engine_clear_pause_reason();
    active_started_ms = millis();
    pause_started_ms = 0UL;
    paused_total_ms = 0UL;

    if (engine_config.control_mode == RECORD_CONTROL_AUTO)
    {
        /* WAV only needs activity monitoring; LEP/L16 already own edge capture. */
        if (engine_config.format == FILE_FORMAT_WAV)
        {
            write_edge_monitor_begin_watch();
        }
        auto_last_edge_count = write_edge_monitor_get_edge_count();
        auto_last_activity_ms = active_started_ms;
    }

    mz_sense_set(false);
    return true;
}

static bool record_engine_pause_capture(void)
{
    bool ok = false;
    if (!capture_started || (engine_state != RECORD_ENGINE_RECORDING))
    {
        return false;
    }
    if (engine_config.format == FILE_FORMAT_WAV)
    {
        ok = wav_record_engine_pause();
    }
    else
    {
        ok = edge_record_engine_pause();
    }
    if (!ok)
    {
        record_engine_set_error_P(PSTR("REC PAUSE"));
        return false;
    }
    pause_started_ms = millis();
    engine_state = RECORD_ENGINE_PAUSED;
    return true;
}

static bool record_engine_resume_capture(void)
{
    bool ok = false;
    if (!capture_started || (engine_state != RECORD_ENGINE_PAUSED))
    {
        return false;
    }
    if (engine_config.format == FILE_FORMAT_WAV)
    {
        ok = wav_record_engine_resume();
    }
    else
    {
        ok = edge_record_engine_resume();
    }
    if (!ok)
    {
        record_engine_set_error_P(PSTR("REC RESUME"));
        return false;
    }
    if (pause_started_ms != 0UL)
    {
        paused_total_ms += millis() - pause_started_ms;
        pause_started_ms = 0UL;
    }
    engine_state = RECORD_ENGINE_RECORDING;
    record_engine_clear_pause_reason();

    /* A user resume in AUTO must start a fresh idle interval; otherwise a long
       manual pause would immediately finalize the recording. */
    if (engine_config.control_mode == RECORD_CONTROL_AUTO)
    {
        auto_last_edge_count = write_edge_monitor_get_edge_count();
        auto_last_activity_ms = millis();
    }
    return true;
}

static void record_engine_pause_for_motor(void)
{
    if (record_engine_pause_capture())
    {
        paused_by_user = false;
        paused_by_motor = (engine_state == RECORD_ENGINE_PAUSED);
    }
}

static void record_engine_pause_for_user(void)
{
    if (record_engine_pause_capture())
    {
        paused_by_motor = false;
        paused_by_user = (engine_state == RECORD_ENGINE_PAUSED);
    }
}

static void record_engine_resume_from_user(void)
{
    paused_by_user = false;

    if ((engine_config.control_mode == RECORD_CONTROL_MOTOR) && !mz_motor_get())
    {
        paused_by_motor = true;
        return;
    }

    paused_by_motor = false;
    (void)record_engine_resume_capture();
}

void record_engine_init(void)
{
    wav_record_engine_init();
    edge_record_engine_init();
    engine_state = RECORD_ENGINE_STOPPED;
    engine_config.format = FILE_FORMAT_WAV;
    engine_config.wav_sample_rate = 22050UL;
    engine_config.control_mode = RECORD_CONTROL_MOTOR;
    engine_directory[0] = '\0';
    engine_error[0] = '\0';
    capture_started = false;
    record_engine_clear_pause_reason();
    active_started_ms = 0UL;
    pause_started_ms = 0UL;
    paused_total_ms = 0UL;
    cancelled_file_removed = false;
    auto_last_edge_count = 0U;
    auto_last_activity_ms = 0UL;
    write_edge_monitor_stop();
    mz_sense_set(true);
}

bool record_engine_start(const char *directory_path, const record_engine_config_t *config)
{
    record_engine_init();

    if ((directory_path == NULL) || (config == NULL))
    {
        record_engine_set_error_P(PSTR("REC ARG"));
        return false;
    }

    strncpy(engine_directory, directory_path, sizeof(engine_directory) - 1U);
    engine_directory[sizeof(engine_directory) - 1U] = '\0';
    engine_config = *config;

    if (engine_config.control_mode == RECORD_CONTROL_MANUAL)
    {
        mz_sense_set(false);
        return record_engine_begin_capture();
    }

    /* MOTOR/AUTO do not create a file yet, but show the same future
       RECxxxx name as MANUAL. The name is scanned only; no SD file is opened. */
    {
        bool preview_ok = (engine_config.format == FILE_FORMAT_WAV) ?
            wav_record_engine_preview_filename(engine_directory) :
            edge_record_engine_preview_filename(engine_directory, engine_config.format);

        if (!preview_ok)
        {
            const char *error = (engine_config.format == FILE_FORMAT_WAV) ?
                wav_record_engine_get_error_text() : edge_record_engine_get_error_text();
            record_engine_set_error(error);
            return false;
        }
    }

    mz_sense_set(false);

    /* MOTOR and AUTO are armed without allocating/creating a file yet. */
    engine_state = RECORD_ENGINE_ARMED;
    if (engine_config.control_mode == RECORD_CONTROL_AUTO)
    {
        write_edge_monitor_arm_auto_trigger();
    }
    return true;
}

void record_engine_service(void)
{
    if (engine_state == RECORD_ENGINE_ARMED)
    {
        if ((engine_config.control_mode == RECORD_CONTROL_MOTOR && mz_motor_get()) ||
            (engine_config.control_mode == RECORD_CONTROL_AUTO &&
             write_edge_monitor_take_auto_trigger()))
        {
            (void)record_engine_begin_capture();
        }
        return;
    }

    if (engine_state == RECORD_ENGINE_RECORDING || engine_state == RECORD_ENGINE_PAUSED)
    {
        if (engine_config.control_mode == RECORD_CONTROL_MOTOR)
        {
            if ((engine_state == RECORD_ENGINE_RECORDING) && !mz_motor_get())
            {
                record_engine_pause_for_motor();
            }
            else if (engine_state == RECORD_ENGINE_PAUSED)
            {
                /* User pause wins over MOTOR HIGH. */
                if (paused_by_user)
                {
                    /* Stay paused until the user explicitly resumes. */
                }
                else if (!mz_motor_get())
                {
                    paused_by_motor = true;
                }
                else if (paused_by_motor)
                {
                    paused_by_motor = false;
                    (void)record_engine_resume_capture();
                }
            }
        }
    }

    if ((engine_config.control_mode == RECORD_CONTROL_AUTO) &&
        (engine_state == RECORD_ENGINE_RECORDING))
    {
        uint16_t edge_count = write_edge_monitor_get_edge_count();
        uint32_t now = millis();
        if (edge_count != auto_last_edge_count)
        {
            auto_last_edge_count = edge_count;
            auto_last_activity_ms = now;
        }
        else if ((uint32_t)(now - auto_last_activity_ms) >= AUTO_RECORD_IDLE_STOP_MS)
        {
            record_engine_request_stop();
        }
    }

    if (engine_config.format == FILE_FORMAT_WAV)
    {
        wav_record_engine_service();
        switch (wav_record_engine_get_state())
        {
            case WAV_RECORD_ENGINE_FINALIZING: engine_state = RECORD_ENGINE_FINALIZING; break;
            case WAV_RECORD_ENGINE_FINISHED:
                engine_state = RECORD_ENGINE_FINISHED;
                record_engine_clear_pause_reason();
                write_edge_monitor_stop();
                mz_sense_set(true);
                break;
            case WAV_RECORD_ENGINE_ERROR: record_engine_set_error(wav_record_engine_get_error_text()); break;
            case WAV_RECORD_ENGINE_PAUSED: engine_state = RECORD_ENGINE_PAUSED; break;
            case WAV_RECORD_ENGINE_RECORDING:
                if (engine_state != RECORD_ENGINE_FINALIZING)
                {
                    engine_state = RECORD_ENGINE_RECORDING;
                    record_engine_clear_pause_reason();
                }
                break;
            default: break;
        }
    }
    else if (is_edge_format(engine_config.format))
    {
        edge_record_engine_service();
        switch (edge_record_engine_get_state())
        {
            case EDGE_RECORD_ENGINE_FINALIZING: engine_state = RECORD_ENGINE_FINALIZING; break;
            case EDGE_RECORD_ENGINE_FINISHED:
                engine_state = RECORD_ENGINE_FINISHED;
                record_engine_clear_pause_reason();
                write_edge_monitor_stop();
                mz_sense_set(true);
                break;
            case EDGE_RECORD_ENGINE_ERROR: record_engine_set_error(edge_record_engine_get_error_text()); break;
            case EDGE_RECORD_ENGINE_PAUSED: engine_state = RECORD_ENGINE_PAUSED; break;
            case EDGE_RECORD_ENGINE_RECORDING:
                if (engine_state != RECORD_ENGINE_FINALIZING)
                {
                    engine_state = RECORD_ENGINE_RECORDING;
                    record_engine_clear_pause_reason();
                }
                break;
            default: break;
        }
    }
}

void record_engine_toggle_pause(void)
{
    if (engine_state == RECORD_ENGINE_ARMED)
    {
        if (record_engine_begin_capture() &&
            (engine_config.control_mode == RECORD_CONTROL_MOTOR) && !mz_motor_get())
        {
            record_engine_pause_for_motor();
        }
        return;
    }

    if (engine_state == RECORD_ENGINE_RECORDING)
    {
        record_engine_pause_for_user();
        return;
    }

    if (engine_state == RECORD_ENGINE_PAUSED)
    {
        record_engine_resume_from_user();
    }
}

void record_engine_request_stop(void)
{
    if (engine_state == RECORD_ENGINE_ARMED)
    {
        record_engine_cancel();
        return;
    }
    if (engine_state == RECORD_ENGINE_RECORDING || engine_state == RECORD_ENGINE_PAUSED)
    {
        if (engine_config.format == FILE_FORMAT_WAV)
        {
            wav_record_engine_request_stop();
        }
        else
        {
            edge_record_engine_request_stop();
        }
        engine_state = RECORD_ENGINE_FINALIZING;
        record_engine_clear_pause_reason();
        write_edge_monitor_stop();
    }
}

void record_engine_cancel(void)
{
    /* A started capture always owns a just-created RECxxxx file. */
    cancelled_file_removed = capture_started;

    if (capture_started)
    {
        if (engine_config.format == FILE_FORMAT_WAV)
        {
            wav_record_engine_cancel();
        }
        else if (is_edge_format(engine_config.format))
        {
            edge_record_engine_cancel();
        }
    }

    /* Keep the result visible; short LEFT acknowledges it and returns. */
    engine_state = RECORD_ENGINE_CANCELLED;
    capture_started = false;
    record_engine_clear_pause_reason();
    write_edge_monitor_stop();
    mz_sense_set(true);
}

record_engine_state_t record_engine_get_state(void) { return engine_state; }
file_format_t record_engine_get_format(void) { return engine_config.format; }
record_control_mode_t record_engine_get_control_mode(void) { return engine_config.control_mode; }
const char *record_engine_get_filename(void)
{
    const char *name = (engine_config.format == FILE_FORMAT_WAV) ?
        wav_record_engine_get_filename() : edge_record_engine_get_filename();

    return (name != NULL && name[0] != '\0') ? name : NULL;
}
const char *record_engine_get_error_text(void) { return engine_error; }
bool record_engine_cancelled_file_removed(void) { return cancelled_file_removed; }
uint8_t record_engine_get_buffer_fill_percent(void)
{
    if (!capture_started) return 100U;
    return (engine_config.format == FILE_FORMAT_WAV) ?
        (uint8_t)(100U - wav_record_engine_get_buffer_fill_percent()) :
        edge_record_engine_get_buffer_headroom_percent();
}
uint32_t record_engine_get_elapsed_seconds(void)
{
    uint32_t end_ms;
    if (!capture_started) return 0UL;
    end_ms = (engine_state == RECORD_ENGINE_PAUSED && pause_started_ms != 0UL) ?
        pause_started_ms : millis();
    if (end_ms < active_started_ms + paused_total_ms) return 0UL;
    return (end_ms - active_started_ms - paused_total_ms) / 1000UL;
}
char record_engine_get_pause_indicator(void)
{
    if (engine_state != RECORD_ENGINE_PAUSED) return '\0';
    if (paused_by_user) return 'U';
    if (paused_by_motor) return 'M';
    return '\0';
}
