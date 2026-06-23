#ifndef SD2CMT2_MZIO_H
#define SD2CMT2_MZIO_H

#include <stdbool.h>

void mzio_init(void);

void mz_read_set(bool level);
void mz_sense_set(bool level);
void mz_led_set(bool level);

bool mz_read_get(void);
bool mz_sense_get(void);
bool mz_led_get(void);

bool mz_motor_get(void);
bool mz_write_get(void);

#endif