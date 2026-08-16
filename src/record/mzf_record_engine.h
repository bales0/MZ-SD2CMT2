#ifndef SD2CMT2_MZF_RECORD_ENGINE_H
#define SD2CMT2_MZF_RECORD_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MZF_RECORD_ENGINE_STOPPED = 0,
    MZF_RECORD_ENGINE_RECORDING,
    MZF_RECORD_ENGINE_PAUSED,
    MZF_RECORD_ENGINE_FINALIZING,
    MZF_RECORD_ENGINE_FINISHED,
    MZF_RECORD_ENGINE_ERROR
} mzf_record_engine_state_t;

void mzf_record_engine_init(void);
bool mzf_record_engine_preview_filename(const char *directory_path);
bool mzf_record_engine_start(const char *directory_path);
bool mzf_record_engine_pause(void);
bool mzf_record_engine_resume(void);
void mzf_record_engine_request_stop(void);
void mzf_record_engine_cancel(void);
void mzf_record_engine_service(void);

mzf_record_engine_state_t mzf_record_engine_get_state(void);
const char *mzf_record_engine_get_filename(void);
const char *mzf_record_engine_get_error_text(void);
uint8_t mzf_record_engine_get_buffer_headroom_percent(void);

#endif
