#ifndef SD2CMT2_MZ_TAPE_DECODER_H
#define SD2CMT2_MZ_TAPE_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#define MZ_TAPE_HEADER_BYTES 128U

typedef enum
{
    MZ_TAPE_DECODER_EVENT_NONE = 0,
    MZ_TAPE_DECODER_EVENT_HEADER_VALID,
    MZ_TAPE_DECODER_EVENT_DATA_BYTE,
    MZ_TAPE_DECODER_EVENT_BLOCK_VALID,
    MZ_TAPE_DECODER_EVENT_BLOCK_INVALID
} mz_tape_decoder_event_type_t;

typedef struct
{
    mz_tape_decoder_event_type_t type;
    uint8_t value;
    uint32_t byte_index;
    uint16_t calculated_checksum;
    uint16_t recorded_checksum;
    uint16_t leader_pulses;
    uint8_t copy_index;
} mz_tape_decoder_event_t;

/* Singleton foreground decoder. RECORD modes are mutually exclusive, so one
   instance avoids allocating duplicate 128-byte acquisition buffers. */
void mz_tape_decoder_begin_header(void);
void mz_tape_decoder_start_data(uint32_t byte_count, bool invert_pulse_phase);
/* Retry after an invalid NORMAL block. Native MZ700 may continue after its
   256-short separator; MZ800 can be reacquired from a fresh full leader. */
void mz_tape_decoder_start_recovery_data(uint32_t byte_count);
void mz_tape_decoder_break_signal(void);
void mz_tape_decoder_stop(void);

/* level is the physical level held for this elapsed half interval. */
void mz_tape_decoder_feed_interval(uint16_t duration_units, uint8_t level);
bool mz_tape_decoder_take_event(mz_tape_decoder_event_t *event);

/* Valid until begin_header() is called again. Raw Sharp bytes are unchanged. */
const uint8_t *mz_tape_decoder_get_header(void);
uint8_t mz_tape_decoder_get_pulse_start_level(void);

#endif
