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
    EDGE_RECORD_DRIVER_BAD_ARGUMENT
} edge_record_driver_state_t;

/* One invalid LEP byte is reserved internally for extended intervals. */
#define EDGE_RECORD_EXTENDED_TOKEN 0x80U
#define EDGE_RECORD_EXTENDED_TOKEN_BYTES 6U

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

#endif
