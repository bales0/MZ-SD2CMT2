#pragma once

#include <Arduino.h>
#include "../formats/file_format.h"
#include "../ui/menu.h"

typedef enum
{
    PLAY_ENGINE_STATE_STOPPED = 0,
    PLAY_ENGINE_STATE_READY,
    PLAY_ENGINE_STATE_RUNNING,
    PLAY_ENGINE_STATE_PAUSED,
    PLAY_ENGINE_STATE_ERROR
} play_engine_state_t;

typedef struct
{
    const char *full_path;
    file_format_t format;
    menu_play_mode_t play_mode;
    bool invert_signal;
} play_engine_config_t;

void play_engine_init(void);
bool play_engine_prepare(const play_engine_config_t *config);
bool play_engine_start(void);
bool play_engine_pause(void);
bool play_engine_resume(void);
void play_engine_stop(void);

/* Foreground only: one bounded SD refill and normal EOF/error synchronization. */
void play_engine_service(void);

play_engine_state_t play_engine_get_state(void);

const char *play_engine_get_error_text(void);

uint8_t play_engine_get_output_pin(void);

/* WAV playback progress, measured in emitted source sample periods. */
uint32_t play_engine_get_played_samples(void);
uint32_t play_engine_get_total_samples(void);

/* Current prepared FIFO fill, as 0..100 %. */
uint8_t play_engine_get_buffer_fill_percent(void);

/*
    Software jitter statistics are disabled in the sample ISR. D11/OC1A timing
    marker is the accurate jitter probe; this accessor remains for UI ABI.
*/
uint16_t play_engine_get_jitter_ticks(void);
