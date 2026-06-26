#pragma once

#include <Arduino.h>
#include "../formats/file_format.h"
#include "../ui/menu.h"

#define PLAY_CONTROLLER_NAME_MAX 64
#define PLAY_CONTROLLER_PATH_MAX 160

typedef enum
{
    PLAY_CONTROLLER_STATE_READY = 0,
    PLAY_CONTROLLER_STATE_PLAYING,
    PLAY_CONTROLLER_STATE_PAUSED,
    PLAY_CONTROLLER_STATE_ERROR
} play_controller_state_t;

typedef struct
{
    const char *filename;
    const char *full_path;
    file_format_t format;
    bool invert_signal;
    play_control_mode_t control_mode;
    bool waiting_for_motor;

    /* These are mutually exclusive while state == PAUSED. */
    bool paused_by_motor;
    bool paused_by_user;

    play_controller_state_t state;
    const char *error_text;

    /* WAV/MZF/MZT/M12 show active transport time and nominal duration. */
    uint32_t elapsed_ms;
    uint32_t total_duration_ms;

    /* LEP/L16 show immediate source-byte progress instead of a scanned duration. */
    bool progress_is_percent;
    uint8_t progress_percent;

    /* Prepared sample FIFO fill 0..100 %. */
    uint8_t buffer_fill_percent;
} play_controller_view_t;

void play_controller_init(void);
void play_controller_start_session(const char *filename,
                                   const char *full_path,
                                   bool invert_signal,
                                   play_control_mode_t control_mode);
void play_controller_toggle_play_pause(void);
void play_controller_stop(void);

/* Synchronizes UI state with asynchronous transport completion/error. */
void play_controller_service(void);

void play_controller_get_view(play_controller_view_t *view);
const char* play_controller_get_filename(void);
const char* play_controller_get_full_path(void);
file_format_t play_controller_get_format(void);
bool play_controller_get_invert_signal(void);
play_control_mode_t play_controller_get_control_mode(void);
play_controller_state_t play_controller_get_state(void);
