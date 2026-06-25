#include <Arduino.h>
#include <string.h>

#include "play_controller.h"
#include "play_engine.h"
#include "../drivers/mzio.h"
#include "../drivers/wav_playback_driver.h"

static char session_filename[PLAY_CONTROLLER_NAME_MAX];
static char session_full_path[PLAY_CONTROLLER_PATH_MAX];
static file_format_t session_format = FILE_FORMAT_UNKNOWN;
static menu_play_mode_t session_play_mode = MENU_PLAY_MODE_NORMAL;
static bool session_invert_signal = false;
static play_control_mode_t session_control_mode = PLAY_CONTROL_MOTOR;
static play_controller_state_t session_state = PLAY_CONTROLLER_STATE_READY;

/* MOTOR mode is armed only once when a file is selected.  EOF must not loop. */
static bool waiting_for_motor = false;
static bool manual_override = false;
static bool paused_by_motor = false;

static bool play_controller_can_start(void)
{
    return session_format != FILE_FORMAT_UNKNOWN;
}

static uint8_t play_controller_percent(uint32_t played, uint32_t total)
{
    uint32_t percent;
    if (total == 0UL) return 0U;
    percent = (total >= 100UL) ? (played / (total / 100UL)) :
                                 ((played * 100UL) / total);
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

static bool play_controller_start_engine(void)
{
    if (!play_engine_start())
    {
        session_state = PLAY_CONTROLLER_STATE_ERROR;
        mz_sense_set(true);
        return false;
    }
    session_state = PLAY_CONTROLLER_STATE_PLAYING;
    return true;
}

static bool play_controller_pause_engine(void)
{
    if (!play_engine_pause())
    {
        session_state = PLAY_CONTROLLER_STATE_ERROR;
        mz_sense_set(true);
        return false;
    }
    if (play_engine_get_state() == PLAY_ENGINE_STATE_READY)
    {
        /* ULTRA FAST may finish while consuming the jumper-block MOTOR edge. */
        session_state = PLAY_CONTROLLER_STATE_READY;
        waiting_for_motor = false;
        paused_by_motor = false;
        return true;
    }

    session_state = PLAY_CONTROLLER_STATE_PAUSED;
    return true;
}

static bool play_controller_resume_engine(void)
{
    if (!play_engine_resume())
    {
        session_state = PLAY_CONTROLLER_STATE_ERROR;
        mz_sense_set(true);
        return false;
    }
    session_state = PLAY_CONTROLLER_STATE_PLAYING;
    return true;
}

void play_controller_init(void)
{
    session_filename[0] = '\0';
    session_full_path[0] = '\0';
    session_format = FILE_FORMAT_UNKNOWN;
    session_play_mode = MENU_PLAY_MODE_NORMAL;
    session_invert_signal = false;
    session_control_mode = PLAY_CONTROL_MOTOR;
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    manual_override = false;
    paused_by_motor = false;
    play_engine_init();
}

void play_controller_start_session(const char *filename,
                                   const char *full_path,
                                   menu_play_mode_t play_mode,
                                   bool invert_signal,
                                   play_control_mode_t control_mode)
{
    if (filename == NULL) session_filename[0] = '\0';
    else
    {
        strncpy(session_filename, filename, sizeof(session_filename) - 1U);
        session_filename[sizeof(session_filename) - 1U] = '\0';
    }

    if (full_path == NULL) session_full_path[0] = '\0';
    else
    {
        strncpy(session_full_path, full_path, sizeof(session_full_path) - 1U);
        session_full_path[sizeof(session_full_path) - 1U] = '\0';
    }

    session_format = file_format_detect_from_name(session_filename);
    session_play_mode = play_mode;
    session_invert_signal = invert_signal;
    session_control_mode = control_mode;
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    manual_override = false;
    paused_by_motor = false;

    if (session_format == FILE_FORMAT_UNKNOWN)
    {
        play_engine_stop();
        return;
    }

    play_engine_config_t config;
    config.full_path = session_full_path;
    config.format = session_format;
    config.play_mode = session_play_mode;
    config.invert_signal = session_invert_signal;

    if (!play_engine_prepare(&config))
    {
        session_state = PLAY_CONTROLLER_STATE_ERROR;
        return;
    }

    if (session_control_mode == PLAY_CONTROL_MOTOR)
    {
        /* Signal that the CMT session is ready while awaiting MOTOR HIGH. */
        waiting_for_motor = true;
        mz_sense_set(false);
    }
    else
    {
        /* MANUAL begins immediately; SELECT is then only pause/resume. */
        (void)play_controller_start_engine();
    }
}

void play_controller_toggle_play_pause(void)
{
    if (!play_controller_can_start()) return;

    /* Any user SELECT becomes a lasting manual transport override. */
    manual_override = true;
    waiting_for_motor = false;
    paused_by_motor = false;

    switch (session_state)
    {
        case PLAY_CONTROLLER_STATE_READY:
            (void)play_controller_start_engine();
            break;

        case PLAY_CONTROLLER_STATE_PLAYING:
            (void)play_controller_pause_engine();
            break;

        case PLAY_CONTROLLER_STATE_PAUSED:
            (void)play_controller_resume_engine();
            break;

        case PLAY_CONTROLLER_STATE_ERROR:
        default:
            play_engine_stop();
            session_state = PLAY_CONTROLLER_STATE_READY;
            mz_sense_set(true);
            break;
    }
}

void play_controller_stop(void)
{
    play_engine_stop();
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    manual_override = false;
    paused_by_motor = false;
}

void play_controller_service(void)
{
    play_engine_state_t engine_state;

    play_engine_service();
    engine_state = play_engine_get_state();

    switch (engine_state)
    {
        case PLAY_ENGINE_STATE_RUNNING:
            session_state = PLAY_CONTROLLER_STATE_PLAYING;
            break;

        case PLAY_ENGINE_STATE_PAUSED:
            session_state = PLAY_CONTROLLER_STATE_PAUSED;
            break;

        case PLAY_ENGINE_STATE_ERROR:
            session_state = PLAY_CONTROLLER_STATE_ERROR;
            waiting_for_motor = false;
            break;

        case PLAY_ENGINE_STATE_READY:
        case PLAY_ENGINE_STATE_STOPPED:
        default:
            if ((session_state == PLAY_CONTROLLER_STATE_PLAYING) ||
                (session_state == PLAY_CONTROLLER_STATE_PAUSED))
            {
                /* EOF returns READY but must not auto-repeat while MOTOR stays high. */
                session_state = PLAY_CONTROLLER_STATE_READY;
                waiting_for_motor = false;
                paused_by_motor = false;
            }
            break;
    }

    if ((session_control_mode != PLAY_CONTROL_MOTOR) || manual_override ||
        (session_state == PLAY_CONTROLLER_STATE_ERROR))
    {
        return;
    }

    if (waiting_for_motor)
    {
        if (mz_motor_get() && (play_engine_get_state() == PLAY_ENGINE_STATE_READY))
        {
            if (play_controller_start_engine()) waiting_for_motor = false;
        }
        return;
    }

    if ((session_state == PLAY_CONTROLLER_STATE_PLAYING) && !mz_motor_get())
    {
        paused_by_motor = true;
        (void)play_controller_pause_engine();
    }
    else if ((session_state == PLAY_CONTROLLER_STATE_PAUSED) &&
             paused_by_motor && mz_motor_get())
    {
        paused_by_motor = false;
        (void)play_controller_resume_engine();
    }
}

void play_controller_get_view(play_controller_view_t *view)
{
    uint32_t played;
    uint32_t total;
    if (view == NULL) return;

    played = play_engine_get_played_samples();
    total = play_engine_get_total_samples();
    view->filename = session_filename;
    view->full_path = session_full_path;
    view->format = session_format;
    view->play_mode = session_play_mode;
    view->invert_signal = session_invert_signal;
    view->control_mode = session_control_mode;
    view->waiting_for_motor = waiting_for_motor;
    view->manual_override = manual_override;
    view->state = session_state;
    view->error_text = play_engine_get_error_text();
    view->output_pin = play_engine_get_output_pin();
    view->played_samples = played;
    view->total_samples = total;
    view->progress_percent = play_controller_percent(played, total);
    view->buffer_fill_percent = play_engine_get_buffer_fill_percent();
    view->jitter_ticks = play_engine_get_jitter_ticks();
    view->timing_marker_pin = WAV_PLAYBACK_TIMING_MARKER_ENABLE ?
        WAV_PLAYBACK_TIMING_MARKER_PIN : 0U;
}

const char* play_controller_get_filename(void) { return session_filename; }
const char* play_controller_get_full_path(void) { return session_full_path; }
file_format_t play_controller_get_format(void) { return session_format; }
menu_play_mode_t play_controller_get_play_mode(void) { return session_play_mode; }
bool play_controller_get_invert_signal(void) { return session_invert_signal; }
play_control_mode_t play_controller_get_control_mode(void) { return session_control_mode; }
play_controller_state_t play_controller_get_state(void) { return session_state; }
