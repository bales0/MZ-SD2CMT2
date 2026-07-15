#ifndef SD2CMT2_MZF_ULTRAFAST_H
#define SD2CMT2_MZF_ULTRAFAST_H

#include <stdbool.h>
#include <stdint.h>

#include "../formats/file_format.h"

/*
   Test-tuning addresses for the small Z80 ultrafast receiver.
   The automatic selector uses LOW first, then HIGH, only when the chosen
   receiver range does not overlap the MZF payload destination range.
*/
#ifndef MZF_ULTRAFAST_LOW_LOAD_ADDR
#define MZF_ULTRAFAST_LOW_LOAD_ADDR 0x1100U
#endif

#ifndef MZF_ULTRAFAST_HIGH_LOAD_ADDR
#define MZF_ULTRAFAST_HIGH_LOAD_ADDR 0xC000U
#endif

#ifndef MZF_ULTRAFAST_PUMP_BYTES
#define MZF_ULTRAFAST_PUMP_BYTES 64U
#endif

typedef enum
{
    MZF_ULTRAFAST_VARIANT_NONE = 0,
    MZF_ULTRAFAST_VARIANT_LOW,
    MZF_ULTRAFAST_VARIANT_HIGH
} mzf_ultrafast_variant_t;

void mzf_ultrafast_reset(void);

bool mzf_ultrafast_prepare(file_format_t format,
                           bool enabled,
                           const uint8_t *header,
                           uint32_t file_size,
                           uint32_t data_offset);

bool mzf_ultrafast_is_active(void);
mzf_ultrafast_variant_t mzf_ultrafast_get_variant(void);

uint16_t mzf_ultrafast_get_loader_size(void);
void mzf_ultrafast_patch_loader_header(uint8_t *header);
uint16_t mzf_ultrafast_build_loader(uint8_t *destination, uint16_t capacity);

bool mzf_ultrafast_begin(void);
bool mzf_ultrafast_pump(uint8_t max_bytes);
bool mzf_ultrafast_is_finished(void);

uint8_t mzf_ultrafast_get_progress_percent(void);
const char *mzf_ultrafast_get_error_text(void);

#endif
