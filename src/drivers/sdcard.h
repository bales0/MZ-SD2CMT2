#ifndef SD2CMT2_SDCARD_H
#define SD2CMT2_SDCARD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    char name[32];
    bool is_dir;
    uint32_t size;

} sdcard_entry_t;

void sdcard_early_prepare_pins(void);

bool sdcard_init(void);

bool sdcard_is_mounted(void);

uint16_t sdcard_count_entries(const char *path, uint16_t max_entries);

bool sdcard_read_entry_by_index(const char *path, uint16_t index, sdcard_entry_t *entry);

uint16_t sdcard_count_root_entries(uint16_t max_entries);

bool sdcard_read_root_entry_by_index(uint16_t index, sdcard_entry_t *entry);

const char *sdcard_last_error(void);

uint8_t sdcard_last_error_code(void);

uint8_t sdcard_last_error_data(void);

#endif