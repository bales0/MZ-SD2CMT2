#include <Arduino.h>
#include <string.h>

#include "play_controller.h"
#include "play_engine.h"
#include "../drivers/mzio.h"
#include "../drivers/wav_playback_driver.h"

static char session_filename[PLAY_CONTROLLER_NAME_MAX];
static char session_full_path[PLAY_CONTROLLER_PATH_MAX];
static file_format_t session_format = FILE_FORMAT_UNKNOWN;
static bool session_invert_signal = false;
static play_control_mode_t session_control_mode = PLAY_CONTROL_MOTOR;
static play_controller_state_t session_state = PLAY_CONTROLLER_STATE_READY;

/* MOTOR mode is armed only once when a file is selected. EOF must not loop. */
static bool waiting_for_motor = false;

/* Manual pause has priority over a MOTOR release. Once the user resumes, the
   current MOTOR level immediately owns the transport again. */
static bool paused_by_motor = false;
static bool paused_by_user = false;

static bool play_controller_can_start(void)
{
    return session_format != FILE_FORMAT_UNKNOWN;
}

static void play_controller_clear_pause_reason(void)
{
    paused_by_motor = false;
    paused_by_user = false;
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
    play_controller_clear_pause_reason();
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
        session_state = PLAY_CONTROLLER_STATE_READY;
        waiting_for_motor = false;
        play_controller_clear_pause_reason();
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
    play_controller_clear_pause_reason();
    return true;
}

static void play_controller_pause_for_motor(void)
{
    if (play_controller_pause_engine())
    {
        paused_by_user = false;
        paused_by_motor = (session_state == PLAY_CONTROLLER_STATE_PAUSED);
    }
}

static void play_controller_pause_for_user(void)
{
    if (play_controller_pause_engine())
    {
        paused_by_motor = false;
        paused_by_user = (session_state == PLAY_CONTROLLER_STATE_PAUSED);
    }
}

/* A user resume only removes the user lock. MOTOR is immediately consulted
   again, so a still-low MOTOR keeps the transport paused and labels it M. */
static void play_controller_resume_from_user(void)
{
    paused_by_user = false;

    if ((session_control_mode == PLAY_CONTROL_MOTOR) && !mz_motor_get())
    {
        paused_by_motor = true;
        return;
    }

    paused_by_motor = false;
    (void)play_controller_resume_engine();
}

void play_controller_init(void)
{
    session_filename[0] = '\0';
    session_full_path[0] = '\0';
    session_format = FILE_FORMAT_UNKNOWN;
    session_invert_signal = false;
    session_control_mode = PLAY_CONTROL_MOTOR;
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    play_controller_clear_pause_reason();
    play_engine_init();
}

void play_controller_start_session(const char *filename,
                                   const char *full_path,
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
    session_invert_signal = invert_signal;
    session_control_mode = control_mode;
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    play_controller_clear_pause_reason();

    if (session_format == FILE_FORMAT_UNKNOWN)
    {
        play_engine_stop();
        return;
    }

    play_engine_config_t config;
    config.full_path = session_full_path;
    config.format = session_format;
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

    switch (session_state)
    {
        case PLAY_CONTROLLER_STATE_READY:
            /* SELECT can explicitly start a prepared MOTOR session. It does
               not permanently override MOTOR: a low level pauses it at once. */
            waiting_for_motor = false;
            if (play_controller_start_engine() &&
                (session_control_mode == PLAY_CONTROL_MOTOR) && !mz_motor_get())
            {
                play_controller_pause_for_motor();
            }
            break;

        case PLAY_CONTROLLER_STATE_PLAYING:
            play_controller_pause_for_user();
            break;

        case PLAY_CONTROLLER_STATE_PAUSED:
            play_controller_resume_from_user();
            break;

        case PLAY_CONTROLLER_STATE_ERROR:
        default:
            play_engine_stop();
            session_state = PLAY_CONTROLLER_STATE_READY;
            waiting_for_motor = false;
            play_controller_clear_pause_reason();
            mz_sense_set(true);
            break;
    }
}

void play_controller_stop(void)
{
    /* LEFT/back must release the MZ monitor immediately, even if a transport
       was stopped between driver state transitions. */
    play_engine_stop();
    mz_sense_set(true);
    session_state = PLAY_CONTROLLER_STATE_READY;
    waiting_for_motor = false;
    play_controller_clear_pause_reason();
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
            play_controller_clear_pause_reason();
            break;

        case PLAY_ENGINE_STATE_PAUSED:
            session_state = PLAY_CONTROLLER_STATE_PAUSED;
            break;

        case PLAY_ENGINE_STATE_ERROR:
            session_state = PLAY_CONTROLLER_STATE_ERROR;
            waiting_for_motor = false;
            play_controller_clear_pause_reason();
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
                play_controller_clear_pause_reason();
            }
            break;
    }

    if ((session_control_mode != PLAY_CONTROL_MOTOR) ||
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
        play_controller_pause_for_motor();
    }
    else if (session_state == PLAY_CONTROLLER_STATE_PAUSED)
    {
        /* A user pause is dominant. MOTOR HIGH must not resume it. */
        if (paused_by_user)
        {
            return;
        }

        if (!mz_motor_get())
        {
            paused_by_motor = true;
            return;
        }

        if (paused_by_motor)
        {
            paused_by_motor = false;
            (void)play_controller_resume_engine();
        }
    }
}

void play_controller_get_view(play_controller_view_t *view)
{
    if (view == NULL) return;

    view->filename = session_filename;
    view->full_path = session_full_path;
    view->format = session_format;
    view->invert_signal = session_invert_signal;
    view->control_mode = session_control_mode;
    view->waiting_for_motor = waiting_for_motor;
    view->paused_by_motor = paused_by_motor;
    view->paused_by_user = paused_by_user;
    view->state = session_state;
    view->error_text = play_engine_get_error_text();
    view->elapsed_ms = play_engine_get_elapsed_ms();
    view->total_duration_ms = play_engine_get_total_duration_ms();
    view->progress_is_percent = (session_format == FILE_FORMAT_LEP) ||
                                (session_format == FILE_FORMAT_L16);
    view->progress_percent = play_engine_get_progress_percent();
    view->buffer_fill_percent = play_engine_get_buffer_fill_percent();
}

const char* play_controller_get_filename(void) { return session_filename; }
const char* play_controller_get_full_path(void) { return session_full_path; }
file_format_t play_controller_get_format(void) { return session_format; }
bool play_controller_get_invert_signal(void) { return session_invert_signal; }
play_control_mode_t play_controller_get_control_mode(void) { return session_control_mode; }
play_controller_state_t play_controller_get_state(void) { return session_state; }
