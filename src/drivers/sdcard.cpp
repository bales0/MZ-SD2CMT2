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
*/

#define SD_CHIP_SELECT_PIN 53
#define SD_COMPAT_SS_PIN 10

/* 1 MHz is adequate for directory browsing but too tight for 44.1/48 kHz WAV streaming. */
#ifndef SD2CMT2_SD_SPI_MHZ
#define SD2CMT2_SD_SPI_MHZ 4
#endif

static SdFat sd;
static FsFile sdcard_stream_file;

static bool sdcard_mounted = false;
static const char *sdcard_error = "NOT INIT";

static uint8_t sdcard_error_code = 0;
static uint8_t sdcard_error_data = 0;

static void sdcard_set_card_error(void)
{
    if (sdcard_stream_file.isOpen())
    {
        sdcard_stream_file.close();
    }

    sdcard_mounted = false;
    sdcard_error = "SD CARD ERROR";

    if (sd.card() != NULL)
    {
        sdcard_error_code = sd.card()->errorCode();
        sdcard_error_data = (uint8_t)sd.card()->errorData();
    }
    else
    {
        sdcard_error_code = 0;
        sdcard_error_data = 0;
    }
}

/*
    sdcard_mounted only means that sd.begin() succeeded at some earlier time.
    A card can still be removed afterwards, so browser operations verify the
    card with a lightweight CID command before accessing the filesystem.

    Sequential format readers deliberately do not call this before every read:
    repeated CMD10 commands would reduce stream throughput. They detect an
    actual I/O failure in sdcard_file_read() instead.
*/
static bool sdcard_probe_present(void)
{
    if (!sdcard_mounted || (sd.card() == NULL))
    {
        return false;
    }

    cid_t cid;

    if (!sd.card()->readCID(&cid))
    {
        sdcard_set_card_error();
        return false;
    }

    return true;
}

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

    /*
        A previous successful begin() is not proof that the physical card is
        still inserted. Probe it first; only then reuse the mounted state.
    */
    if (sdcard_mounted && sdcard_probe_present())
    {
        sdcard_error = "OK";
        sdcard_error_code = 0;
        sdcard_error_data = 0;

        return true;
    }

    if (sdcard_stream_file.isOpen())
    {
        sdcard_stream_file.close();
    }

    sdcard_mounted = false;
    sdcard_error = "SD CARD ERROR";
    sdcard_error_code = 0;
    sdcard_error_data = 0;

    delay(100);

    sdcard_send_idle_clocks();

    delay(20);

    if (!sd.begin(SdSpiConfig(
            SD_CHIP_SELECT_PIN,
            SHARED_SPI,
            SD_SCK_MHZ(SD2CMT2_SD_SPI_MHZ))))
    {
        sdcard_set_card_error();
        return false;
    }

    sdcard_mounted = true;

    if (!sdcard_probe_present())
    {
        return false;
    }

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

    if (!sdcard_probe_present())
    {
        return 0;
    }

    FsFile dir;

    if (!dir.open(path))
    {
        if (!sdcard_probe_present())
        {
            return 0;
        }

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

    /*
        openNext() returning false can mean a normal end of directory, but it
        can also be caused by a card removed during the operation. Probe once
        more before reporting a valid directory count.
    */
    if (!sdcard_probe_present())
    {
        return 0;
    }

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

    if (!sdcard_probe_present())
    {
        return false;
    }

    FsFile dir;

    if (!dir.open(path))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }

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

            if (!sdcard_probe_present())
            {
                memset(entry, 0, sizeof(sdcard_entry_t));
                return false;
            }

            sdcard_error = "OK";
            return true;
        }

        current_index++;
        file.close();
    }

    dir.close();

    if (!sdcard_probe_present())
    {
        return false;
    }

    strncpy(entry->name, "NO ENTRY", sizeof(entry->name) - 1);
    entry->name[sizeof(entry->name) - 1] = '\0';
    entry->is_dir = false;
    entry->size = 0;

    sdcard_error = "NO ENTRY";

    return false;
}

/*
    Sequential file stream API for format readers.

    Exactly one sequential source file can be open at a time. That fits the
    current PLAY architecture and prevents a parser from sharing one FsFile
    object with the browser. Do not call this API from an ISR.
*/
bool sdcard_file_open_read(const char *path)
{
    if (path == NULL)
    {
        sdcard_error = "BAD PATH";
        return false;
    }

    if (!sdcard_probe_present())
    {
        return false;
    }

    if (sdcard_stream_file.isOpen())
    {
        sdcard_stream_file.close();
    }

    if (!sdcard_stream_file.open(path, O_RDONLY))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }

        sdcard_error = "FILE OPEN FAIL";
        return false;
    }

    sdcard_error = "OK";
    return true;
}

int16_t sdcard_file_read(void *buffer, uint16_t size)
{
    if ((buffer == NULL) || (size == 0))
    {
        sdcard_error = "BAD ARG";
        return -1;
    }

    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return -1;
    }

    int result = sdcard_stream_file.read(buffer, size);

    if (result < 0)
    {
        sdcard_set_card_error();
        return -1;
    }

    /*
        A zero-byte read before logical EOF is an I/O failure, not normal EOF.
        Probe only in this exceptional path so sequential reading remains fast.
    */
    if ((result == 0) &&
        (sdcard_stream_file.curPosition() < sdcard_stream_file.fileSize()))
    {
        if (!sdcard_probe_present())
        {
            return -1;
        }

        sdcard_error = "FILE READ FAIL";
        return -1;
    }

    return (int16_t)result;
}

bool sdcard_file_seek(uint32_t position)
{
    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return false;
    }

    if (!sdcard_stream_file.seekSet(position))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }

        sdcard_error = "FILE SEEK FAIL";
        return false;
    }

    return true;
}

uint32_t sdcard_file_size(void)
{
    if (!sdcard_stream_file.isOpen())
    {
        return 0;
    }

    return (uint32_t)sdcard_stream_file.fileSize();
}

uint32_t sdcard_file_position(void)
{
    if (!sdcard_stream_file.isOpen())
    {
        return 0;
    }

    return (uint32_t)sdcard_stream_file.curPosition();
}

bool sdcard_file_is_open(void)
{
    return sdcard_stream_file.isOpen();
}

void sdcard_file_close(void)
{
    if (sdcard_stream_file.isOpen())
    {
        sdcard_stream_file.close();
    }
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
