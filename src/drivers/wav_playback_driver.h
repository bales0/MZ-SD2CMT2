#ifndef SD2CMT2_WAV_PLAYBACK_DRIVER_H
#define SD2CMT2_WAV_PLAYBACK_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/*
    Optional Timer1 hardware timing marker for oscilloscope debugging.

    On Arduino Mega 2560 OC1A is digital pin D11. With this enabled, hardware
    toggles D11 exactly at every Timer1 sample compare event. Compare D11 to
    READ D2 on a two-channel scope: their relative displacement is the real
    output-update latency/jitter. The marker costs zero ISR cycles.
*/
#ifndef WAV_PLAYBACK_TIMING_MARKER_ENABLE
#define WAV_PLAYBACK_TIMING_MARKER_ENABLE 1
#endif

#define WAV_PLAYBACK_TIMING_MARKER_PIN 11U

typedef enum
{
    WAV_PLAYBACK_DRIVER_STOPPED = 0,
    WAV_PLAYBACK_DRIVER_READY,
    WAV_PLAYBACK_DRIVER_RUNNING,
    WAV_PLAYBACK_DRIVER_PAUSED,
    WAV_PLAYBACK_DRIVER_FINISHED,
    WAV_PLAYBACK_DRIVER_UNDERRUN,
    WAV_PLAYBACK_DRIVER_BAD_RATE,
    WAV_PLAYBACK_DRIVER_BAD_ARGUMENT
} wav_playback_driver_state_t;

void wav_playback_driver_init(void);

bool wav_playback_driver_prepare(uint32_t sample_rate);

bool wav_playback_driver_start(void);
bool wav_playback_driver_pause(void);
bool wav_playback_driver_resume(void);

void wav_playback_driver_stop(void);

wav_playback_driver_state_t wav_playback_driver_get_state(void);
uint32_t wav_playback_driver_get_sample_rate(void);
uint8_t wav_playback_driver_get_read_pin(void);

/*
    A 16-bit monotonic emission sequence. Foreground code must read it at
    least once per 65535 samples (the normal main loop does so every pass).
*/
uint16_t wav_playback_driver_get_emitted_sequence(void);

/*
    Per-sample software jitter statistics are intentionally disabled: measuring
    them inside every 44.1/48 kHz ISR consumes the timing budget. Use the
    hardware D11 marker for the accurate measurement instead.
*/
uint16_t wav_playback_driver_get_jitter_ticks(void);

#endif
