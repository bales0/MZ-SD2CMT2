#ifndef SD2CMT2_CALIBRATION_STORE_H
#define SD2CMT2_CALIBRATION_STORE_H

#include <stdbool.h>

#include "keypad.h"

bool calibration_store_load_keypad(keypad_calibration_t *calibration);

bool calibration_store_save_keypad(const keypad_calibration_t *calibration);

#endif