#ifndef SD2CMT2_MZF_PLAYBACK_H
#define SD2CMT2_MZF_PLAYBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "../formats/file_format.h"

typedef enum
{
    MZF_PLAYBACK_STOPPED = 0,
    MZF_PLAYBACK_READY,
    MZF_PLAYBACK_RUNNING,
    MZF_PLAYBACK_PAUSED,
    MZF_PLAYBACK_FINISHED,
    MZF_PLAYBACK_UNDERRUN,
    MZF_PLAYBACK_IO_ERROR,
    MZF_PLAYBACK_BAD_FILE
} mzf_playback_state_t;

/* Streams MZF/MZT/M12 with native MZ-800 monitor PWM framing only. */
void mzf_playback_init(void);
bool mzf_playback_prepare(const char *path, file_format_t format);
bool mzf_playback_start(void);
bool mzf_playback_pause(void);
bool mzf_playback_resume(void);
void mzf_playback_stop(void);
void mzf_playback_service(void);

mzf_playback_state_t mzf_playback_get_state(void);
const char *mzf_playback_get_error_text(void);
uint8_t mzf_playback_get_buffer_fill_percent(void);

/* Nominal monitor PWM duration, excluding MOTOR pause gaps between blocks. */
uint32_t mzf_playback_get_total_duration_ms(void);

/* Internal Timer3 compare-B dispatch hook, called only when MZF owns Timer3B. */
bool mzf_playback_timer3_compb_from_isr(void);

#endif
