#ifndef SD2CMT2_RECORD_AUTONAME_H
#define SD2CMT2_RECORD_AUTONAME_H

#include <stdbool.h>
#include <stdint.h>

#include "../formats/file_format.h"

/*
   Foreground-only MZ tape-header detector. WAV feeds chronological packed
   WRITE samples; LEP/L16 feeds the same quantized edge intervals which are
   written to the recording. No decoder work is performed in an ISR.
*/
void record_autoname_begin(bool enabled);
/* Reset on a real discontinuity/overlong signal interval, not on a MOTOR or
   user pause: captured FIFOs may still contain the end of the MZ header. */
void record_autoname_break_signal(void);
void record_autoname_feed_packed_samples(uint8_t packed, uint8_t valid_bits);
void record_autoname_feed_interval(uint16_t duration_units);

bool record_autoname_has_name(void);
const char *record_autoname_get_name(void);

/* Rename the already closed RECxxxx file. Failure keeps the original file. */
bool record_autoname_apply(const char *directory_path, file_format_t format);

#endif
