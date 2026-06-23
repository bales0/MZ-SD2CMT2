#include "calibration_store.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CALIBRATION_STORE_ADDR 0

#define CALIBRATION_MAGIC 0x5344324BUL
#define CALIBRATION_VERSION 1

typedef struct
{
    uint32_t magic;
    uint16_t version;

    keypad_calibration_t keypad;

    uint16_t checksum;

} calibration_record_t;

static uint16_t calibration_store_checksum_record(const calibration_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;

    uint16_t checksum = 0;

    for (size_t i = 0; i < offsetof(calibration_record_t, checksum); i++)
    {
        checksum = (uint16_t)(checksum + bytes[i]);
    }

    return checksum;
}

bool calibration_store_load_keypad(keypad_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return false;
    }

    calibration_record_t record;

    memset(&record, 0, sizeof(record));

    EEPROM.get(CALIBRATION_STORE_ADDR, record);

    if (record.magic != CALIBRATION_MAGIC)
    {
        return false;
    }

    if (record.version != CALIBRATION_VERSION)
    {
        return false;
    }

    uint16_t expected_checksum = calibration_store_checksum_record(&record);

    if (record.checksum != expected_checksum)
    {
        return false;
    }

    *calibration = record.keypad;

    return true;
}

bool calibration_store_save_keypad(const keypad_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return false;
    }

    calibration_record_t record;

    memset(&record, 0, sizeof(record));

    record.magic = CALIBRATION_MAGIC;
    record.version = CALIBRATION_VERSION;
    record.keypad = *calibration;
    record.checksum = 0;

    record.checksum = calibration_store_checksum_record(&record);

    EEPROM.put(CALIBRATION_STORE_ADDR, record);

    return true;
}