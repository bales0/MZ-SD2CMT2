#ifndef SD2CMT2_RECORD_ENGINE_H
#define SD2CMT2_RECORD_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "../formats/file_format.h"
#include "../ui/record_menu.h"

typedef enum
{
    RECORD_ENGINE_STOPPED = 0,
    RECORD_ENGINE_ARMED,
    RECORD_ENGINE_RECORDING,
    RECORD_ENGINE_PAUSED,
    RECORD_ENGINE_FINALIZING,
    RECORD_ENGINE_FINISHED,
    RECORD_ENGINE_CANCELLED,
    RECORD_ENGINE_ERROR
} record_engine_state_t;

typedef struct
{
    file_format_t format;
    uint32_t wav_sample_rate;
    record_control_mode_t control_mode;
} record_engine_config_t;

void record_engine_init(void);
bool record_engine_start(const char *directory_path, const record_engine_config_t *config);
void record_engine_service(void);
void record_engine_toggle_pause(void);
void record_engine_request_stop(void);
void record_engine_cancel(void);

record_engine_state_t record_engine_get_state(void);
file_format_t record_engine_get_format(void);
record_control_mode_t record_engine_get_control_mode(void);
/* Effective source of transport control; SELECT override is MANUAL. */
record_control_mode_t record_engine_get_display_control_mode(void);
const char *record_engine_get_filename(void);
const char *record_engine_get_error_text(void);
bool record_engine_cancelled_file_removed(void);
uint8_t record_engine_get_buffer_fill_percent(void);
uint32_t record_engine_get_elapsed_seconds(void);

#endif
