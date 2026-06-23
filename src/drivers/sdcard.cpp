#include "sdcard.h"

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <string.h>

/*
    SD2CMT2 Reborn - Arduino Mega 2560 SD SPI mapping

    SD MISO -> Arduino 50
    SD MOSI -> Arduino 51
    SD SCK  -> Arduino 52
    SD SS   -> Arduino 53

    SD init je zamerne jednoduchy:
    - CS nastavime HIGH co nejdrive
    - posleme idle clocky
    - SdFat sd.begin()

    Nepouzivame rucni CMD12 hack.
*/

#define SD_CHIP_SELECT_PIN 53
#define SD_COMPAT_SS_PIN 10

static SdFat sd;

static bool sdcard_mounted = false;
static const char *sdcard_error = "NOT INIT";

static uint8_t sdcard_error_code = 0;
static uint8_t sdcard_error_data = 0;

void sdcard_early_prepare_pins(void)
{
    pinMode(SD_CHIP_SELECT_PIN, OUTPUT);
    digitalWrite(SD_CHIP_SELECT_PIN, HIGH);

    pinMode(SD_COMPAT_SS_PIN, OUTPUT);
    digitalWrite(SD_COMPAT_SS_PIN, HIGH);
}

static void sdcard_send_idle_clocks(void)
{
    digitalWrite(SD_CHIP_SELECT_PIN, HIGH);
    digitalWrite(SD_COMPAT_SS_PIN, HIGH);

    SPI.begin();

    SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));

    for (uint8_t i = 0; i < 10; i++)
    {
        SPI.transfer(0xFF);
    }

    SPI.endTransaction();
}

bool sdcard_init(void)
{
    sdcard_early_prepare_pins();

    if (sdcard_mounted)
    {
        sdcard_error = "OK";
        sdcard_error_code = 0;
        sdcard_error_data = 0;

        return true;
    }

    sdcard_error = "INIT FAIL";
    sdcard_error_code = 0;
    sdcard_error_data = 0;

    delay(100);

    sdcard_send_idle_clocks();

    delay(20);

    if (!sd.begin(SdSpiConfig(
            SD_CHIP_SELECT_PIN,
            SHARED_SPI,
            SD_SCK_MHZ(1))))
    {
        sdcard_mounted = false;
        sdcard_error = "BEGIN FAIL";

        sdcard_error_code = sd.card()->errorCode();
        sdcard_error_data = sd.card()->errorData();

        return false;
    }

    sdcard_mounted = true;
    sdcard_error = "OK";

    sdcard_error_code = 0;
    sdcard_error_data = 0;

    return true;
}

bool sdcard_is_mounted(void)
{
    return sdcard_mounted;
}

uint16_t sdcard_count_entries(const char *path, uint16_t max_entries)
{
    if (path == NULL)
    {
        sdcard_error = "BAD PATH";
        return 0;
    }

    if (!sdcard_mounted)
    {
        sdcard_error = "NOT MOUNTED";
        return 0;
    }

    FsFile dir;

    if (!dir.open(path))
    {
        sdcard_error = "DIR FAIL";
        return 0;
    }

    if (!dir.isDir())
    {
        dir.close();
        sdcard_error = "NOT DIR";
        return 0;
    }

    FsFile file;
    uint16_t count = 0;

    while (file.openNext(&dir, O_RDONLY))
    {
        count++;

        file.close();

        if (count >= max_entries)
        {
            break;
        }
    }

    dir.close();

    sdcard_error = "OK";

    return count;
}

bool sdcard_read_entry_by_index(const char *path, uint16_t index, sdcard_entry_t *entry)
{
    if (path == NULL)
    {
        sdcard_error = "BAD PATH";
        return false;
    }

    if (entry == NULL)
    {
        sdcard_error = "BAD ARG";
        return false;
    }

    memset(entry, 0, sizeof(sdcard_entry_t));

    if (!sdcard_mounted)
    {
        sdcard_error = "NOT MOUNTED";
        return false;
    }

    FsFile dir;

    if (!dir.open(path))
    {
        sdcard_error = "DIR FAIL";
        return false;
    }

    if (!dir.isDir())
    {
        dir.close();
        sdcard_error = "NOT DIR";
        return false;
    }

    FsFile file;
    uint16_t current_index = 0;

    while (file.openNext(&dir, O_RDONLY))
    {
        if (current_index == index)
        {
            file.getName(entry->name, sizeof(entry->name));
            entry->name[sizeof(entry->name) - 1] = '\0';

            entry->is_dir = file.isDir();
            entry->size = file.fileSize();

            file.close();
            dir.close();

            sdcard_error = "OK";
            return true;
        }

        current_index++;

        file.close();
    }

    dir.close();

    strncpy(entry->name, "NO ENTRY", sizeof(entry->name) - 1);
    entry->name[sizeof(entry->name) - 1] = '\0';
    entry->is_dir = false;
    entry->size = 0;

    sdcard_error = "NO ENTRY";

    return false;
}

uint16_t sdcard_count_root_entries(uint16_t max_entries)
{
    return sdcard_count_entries("/", max_entries);
}

bool sdcard_read_root_entry_by_index(uint16_t index, sdcard_entry_t *entry)
{
    return sdcard_read_entry_by_index("/", index, entry);
}

const char *sdcard_last_error(void)
{
    return sdcard_error;
}

uint8_t sdcard_last_error_code(void)
{
    return sdcard_error_code;
}

uint8_t sdcard_last_error_data(void)
{
    return sdcard_error_data;
}