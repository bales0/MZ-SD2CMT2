#ifndef SD2CMT2_WAV_PLAYBACK_DRIVER_H
#define SD2CMT2_WAV_PLAYBACK_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/*
    Hardware-timed WAV sample output on Arduino Mega 2560.

    READ is D2 / PE4 / OC3B. Timer3 compare B sets or clears this pin in
    hardware at every sample boundary. The compare-A ISR prepares the next
    output level 192 CPU clocks (12 us) before that hardware edge.

    Optional diagnostic marker: D3 / PE5 / OC3C toggles at exactly the same
    Timer3 count as the D2 hardware sample boundary. It is not connected to
    the CMT interface and is intended only for an oscilloscope probe.
*/
#ifndef WAV_PLAYBACK_TIMING_MARKER_ENABLE
#define WAV_PLAYBACK_TIMING_MARKER_ENABLE 1
#endif

#define WAV_PLAYBACK_TIMING_MARKER_PIN 3U

/* Must remain comfortably below the shortest supported sample period. */
#define WAV_PLAYBACK_TIMER3_PREP_LEAD_TICKS 192U

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
    A monotonic 16-bit count of sample values armed for hardware output.
    Foreground code reads it at least once per 65535 samples.
*/
uint16_t wav_playback_driver_get_emitted_sequence(void);

/*
    D2 timing is now hardware compare timing, so no software-jitter value is
    measured in the hot ISR. Probe D2 and the optional D3 marker instead.
*/
uint16_t wav_playback_driver_get_jitter_ticks(void);

#endif
