#ifndef SD2CMT2_EDGE_RECORD_DRIVER_H
#define SD2CMT2_EDGE_RECORD_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    EDGE_RECORD_DRIVER_STOPPED = 0,
    EDGE_RECORD_DRIVER_READY,
    EDGE_RECORD_DRIVER_RUNNING,
    EDGE_RECORD_DRIVER_PAUSED,
    EDGE_RECORD_DRIVER_OVERRUN,
    EDGE_RECORD_DRIVER_TOO_LONG,
    EDGE_RECORD_DRIVER_BAD_ARGUMENT
} edge_record_driver_state_t;

/*
   Internal ISR-to-foreground stream. It is never written directly to a
   .LEP/.L16 file. The foreground immediately expands it to normal standard
   signed slots and writes those slots into the final file.

   0x10..0xFF: high nibble = one 1..15-unit interval, low nibble = optional
               second 1..15-unit interval. Polarity is implicit because each
               accepted WRITE transition changes the signal polarity.
   0x01,u:     one 16..127-unit interval.
   0x02,u0..u3: one interval larger than 127 units, little-endian uint32.

   The normal 5 kHz paths use only packed nibbles. Long tokens occur only for
   slow pulses and are expanded to a signed slot followed by zero-extension
   bytes directly while RECORD is still active.
*/
#define EDGE_RECORD_TOKEN_UNIT 0x01U
#define EDGE_RECORD_TOKEN_LONG 0x02U
#define EDGE_RECORD_TOKEN_LONG_BYTES 5U

void edge_record_driver_init(void);
bool edge_record_driver_prepare(uint8_t unit_us);
bool edge_record_driver_start(void);
bool edge_record_driver_pause(void);
bool edge_record_driver_resume(void);
void edge_record_driver_stop(void);
void edge_record_driver_abort(void);

/* Called only by write_edge_monitor PCINT1 ISR. */
void edge_record_driver_on_write_edge_from_isr(uint8_t new_level);

edge_record_driver_state_t edge_record_driver_get_state(void);
uint16_t edge_record_driver_available_bytes(void);
bool edge_record_driver_pop_byte(uint8_t *value);
uint8_t edge_record_driver_fill_percent(void);
uint32_t edge_record_driver_get_captured_active_ticks(void);
uint8_t edge_record_driver_get_initial_level(void);

#endif
