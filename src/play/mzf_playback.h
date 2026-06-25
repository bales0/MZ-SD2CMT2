#ifndef SD2CMT2_MZF_PLAYBACK_H
#define SD2CMT2_MZF_PLAYBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "../formats/file_format.h"
#include "../ui/menu.h"

typedef enum
{
    MZF_PLAYBACK_STOPPED = 0,
    MZF_PLAYBACK_READY,
    MZF_PLAYBACK_RUNNING,
    MZF_PLAYBACK_PAUSED,
    MZF_PLAYBACK_FINISHED,
    MZF_PLAYBACK_UNDERRUN,
    MZF_PLAYBACK_IO_ERROR,
    MZF_PLAYBACK_BAD_FILE,
    MZF_PLAYBACK_ULTRA_ERROR
} mzf_playback_state_t;

/*
    Streams Sharp binary tape images without expanding them in SRAM.
    NORMAL emits the monitor-compatible PWM tape framing. ULTRA FAST injects
    the compatible loader used by the original MZ-SD2CMT project, then sends
    the original payload through READ/WRITE/SENSE handshaking.
*/
void mzf_playback_init(void);
/* MZF/MZT/M12 use fixed native Sharp polarity; no invert argument. */
bool mzf_playback_prepare(const char *path,
                          file_format_t format,
                          menu_play_mode_t play_mode);
bool mzf_playback_start(void);
bool mzf_playback_pause(void);
bool mzf_playback_resume(void);
void mzf_playback_stop(void);
void mzf_playback_service(void);

mzf_playback_state_t mzf_playback_get_state(void);
const char *mzf_playback_get_error_text(void);
uint8_t mzf_playback_get_buffer_fill_percent(void);
uint32_t mzf_playback_get_consumed_bytes(void);
uint32_t mzf_playback_get_total_bytes(void);

/* Internal Timer3 compare-B dispatch hook. Called only from ISR context.
   Returns true when NORMAL MZF/MZT transport consumed the compare event. */
bool mzf_playback_timer3_compb_from_isr(void);

#endif
