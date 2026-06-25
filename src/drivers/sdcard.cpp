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

/*
    CMT/LCD hardware in this project does not share the Mega SPI bus. Dedicated
    SPI lets SdFat observe SD busy state instead of blocking a realtime record
    write behind internal card programming. 8 MHz is conservative for common
    5 V SD modules and still has ample bandwidth for 44.1 kHz PCM.
*/
#ifndef SD2CMT2_SD_SPI_MHZ
#define SD2CMT2_SD_SPI_MHZ 8
#endif

#ifndef SD2CMT2_SD_SPI_MODE
#define SD2CMT2_SD_SPI_MODE DEDICATED_SPI
#endif

static SdFat sd;
static FsFile sdcard_stream_file;

static bool sdcard_mounted = false;
static const char *sdcard_error = "NOT INIT";
static uint8_t sdcard_error_code = 0;
static uint8_t sdcard_error_data = 0;

static void sdcard_close_all_files(void)
{
    if (sdcard_stream_file.isOpen())
    {
        sdcard_stream_file.close();
    }
}

static void sdcard_set_card_error(void)
{
    sdcard_close_all_files();
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
    Browser operations probe with CMD10. Stream and conversion reads avoid a
    probe per block because that would add latency; an actual I/O error drops
    the mounted state through sdcard_set_card_error().
*/
static bool sdcard_probe_present(void)
{
    cid_t cid;

    if (!sdcard_mounted || (sd.card() == NULL))
    {
        return false;
    }

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

    if (sdcard_mounted && sdcard_probe_present())
    {
        sdcard_error = "OK";
        sdcard_error_code = 0;
        sdcard_error_data = 0;
        return true;
    }

    sdcard_close_all_files();
    sdcard_mounted = false;
    sdcard_error = "SD CARD ERROR";
    sdcard_error_code = 0;
    sdcard_error_data = 0;

    delay(100);
    sdcard_send_idle_clocks();
    delay(20);

    if (!sd.begin(SdSpiConfig(
            SD_CHIP_SELECT_PIN,
            SD2CMT2_SD_SPI_MODE,
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
    FsFile dir;
    FsFile file;
    uint16_t count = 0;

    if (path == NULL)
    {
        sdcard_error = "BAD PATH";
        return 0;
    }

    if (!sdcard_probe_present())
    {
        return 0;
    }

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

    if (!sdcard_probe_present())
    {
        return 0;
    }

    sdcard_error = "OK";
    return count;
}

bool sdcard_read_entry_by_index(const char *path, uint16_t index, sdcard_entry_t *entry)
{
    FsFile dir;
    FsFile file;
    uint16_t current_index = 0;

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

    strncpy(entry->name, "NO ENTRY", sizeof(entry->name) - 1U);
    entry->name[sizeof(entry->name) - 1U] = '\0';
    entry->is_dir = false;
    entry->size = 0;
    sdcard_error = "NO ENTRY";
    return false;
}

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

bool sdcard_file_open_write(const char *path)
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

    if (!sdcard_stream_file.open(path, O_RDWR | O_CREAT | O_TRUNC))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "FILE CREATE FAIL";
        return false;
    }

    sdcard_error = "OK";
    return true;
}

bool sdcard_file_exists(const char *path)
{
    if ((path == NULL) || !sdcard_mounted)
    {
        return false;
    }

    return sd.exists(path);
}

static char sdcard_ascii_upper(char value)
{
    if ((value >= 'a') && (value <= 'z'))
    {
        return (char)(value - ('a' - 'A'));
    }
    return value;
}

static bool sdcard_parse_record_sequence(const char *name, uint16_t *sequence)
{
    uint16_t value = 0U;
    bool valid_extension;

    if ((name == NULL) || (sequence == NULL) || (strlen(name) != 11U))
    {
        return false;
    }

    if ((sdcard_ascii_upper(name[0]) != 'R') ||
        (sdcard_ascii_upper(name[1]) != 'E') ||
        (sdcard_ascii_upper(name[2]) != 'C') ||
        (name[7] != '.'))
    {
        return false;
    }

    for (uint8_t index = 3U; index <= 6U; ++index)
    {
        if ((name[index] < '0') || (name[index] > '9'))
        {
            return false;
        }
        value = (uint16_t)(value * 10U + (uint16_t)(name[index] - '0'));
    }

    valid_extension =
        ((sdcard_ascii_upper(name[8]) == 'W') &&
         (sdcard_ascii_upper(name[9]) == 'A') &&
         (sdcard_ascii_upper(name[10]) == 'V')) ||
        ((sdcard_ascii_upper(name[8]) == 'L') &&
         (sdcard_ascii_upper(name[9]) == 'E') &&
         (sdcard_ascii_upper(name[10]) == 'P')) ||
        ((sdcard_ascii_upper(name[8]) == 'L') &&
         (name[9] == '1') && (name[10] == '6'));

    if (!valid_extension || (value == 0U) || (value > 9999U))
    {
        return false;
    }

    *sequence = value;
    return true;
}

bool sdcard_ensure_directory(const char *directory_path)
{
    FsFile directory;

    if ((directory_path == NULL) || (directory_path[0] != '/') ||
        (directory_path[1] == '\0'))
    {
        sdcard_error = "BAD DIRECTORY";
        return false;
    }

    if (!sdcard_probe_present())
    {
        return false;
    }

    if (!sd.exists(directory_path) && !sd.mkdir(directory_path, true))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "MKDIR FAIL";
        return false;
    }

    if (!directory.open(directory_path, O_RDONLY))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "DIR FAIL";
        return false;
    }

    if (!directory.isDir())
    {
        directory.close();
        sdcard_error = "NOT DIR";
        return false;
    }

    directory.close();
    sdcard_error = "OK";
    return true;
}

bool sdcard_next_record_sequence(const char *directory_path,
                                 uint16_t *next_sequence)
{
    FsFile directory;
    FsFile entry;
    uint16_t highest = 0U;

    if ((next_sequence == NULL) || !sdcard_ensure_directory(directory_path))
    {
        return false;
    }

    if (!directory.open(directory_path, O_RDONLY) || !directory.isDir())
    {
        if (directory.isOpen())
        {
            directory.close();
        }
        sdcard_error = "DIR FAIL";
        return false;
    }

    while (entry.openNext(&directory, O_RDONLY))
    {
        char name[sizeof(((sdcard_entry_t *)0)->name)];
        uint16_t sequence;

        memset(name, 0, sizeof(name));
        entry.getName(name, sizeof(name));
        name[sizeof(name) - 1U] = '\0';

        if (!entry.isDir() && sdcard_parse_record_sequence(name, &sequence) &&
            (sequence > highest))
        {
            highest = sequence;
        }

        entry.close();
    }

    directory.close();

    if (!sdcard_probe_present())
    {
        return false;
    }

    if (highest >= 9999U)
    {
        sdcard_error = "REC NAMES FULL";
        return false;
    }

    *next_sequence = (uint16_t)(highest + 1U);
    sdcard_error = "OK";
    return true;
}

int16_t sdcard_file_read(void *buffer, uint16_t size)
{
    int result;

    if ((buffer == NULL) || (size == 0U))
    {
        sdcard_error = "BAD ARG";
        return -1;
    }

    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return -1;
    }

    result = sdcard_stream_file.read(buffer, size);

    if (result < 0)
    {
        sdcard_set_card_error();
        return -1;
    }

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

int16_t sdcard_file_write(const void *buffer, uint16_t size)
{
    int result;

    if ((buffer == NULL) || (size == 0U))
    {
        sdcard_error = "BAD ARG";
        return -1;
    }

    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return -1;
    }

    result = (int)sdcard_stream_file.write(buffer, size);

    if (result != (int)size)
    {
        if (!sdcard_probe_present())
        {
            return -1;
        }
        sdcard_error = "FILE WRITE FAIL";
        return -1;
    }

    return (int16_t)result;
}

bool sdcard_file_preallocate(uint32_t length)
{
    if ((length == 0UL) || !sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return false;
    }

    if (!sdcard_stream_file.preAllocate((uint64_t)length))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "PREALLOC FAIL";
        return false;
    }

    sdcard_error = "OK";
    return true;
}

bool sdcard_file_truncate(uint32_t length)
{
    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return false;
    }

    if (!sdcard_stream_file.truncate((uint64_t)length))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "FILE TRUNC FAIL";
        return false;
    }

    sdcard_error = "OK";
    return true;
}

/*
    With DEDICATED_SPI, SdFat can test the card busy signal without forcing a
    blocking write wait. A caller may transmit one 512-byte sector after this
    returns false, then must wait for a later false before sending another.
*/
bool sdcard_file_is_busy(void)
{
    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        return true;
    }

    return sdcard_stream_file.isBusy();
}

bool sdcard_file_sync(void)
{
    if (!sdcard_mounted || !sdcard_stream_file.isOpen())
    {
        sdcard_set_card_error();
        return false;
    }

    if (!sdcard_stream_file.sync())
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "FILE SYNC FAIL";
        return false;
    }

    return true;
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
    return sdcard_stream_file.isOpen() ?
        (uint32_t)sdcard_stream_file.fileSize() : 0UL;
}

uint32_t sdcard_file_position(void)
{
    return sdcard_stream_file.isOpen() ?
        (uint32_t)sdcard_stream_file.curPosition() : 0UL;
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

bool sdcard_file_remove(const char *path)
{
    if ((path == NULL) || !sdcard_mounted)
    {
        sdcard_error = "BAD PATH";
        return false;
    }

    if (!sd.remove(path))
    {
        if (!sdcard_probe_present())
        {
            return false;
        }
        sdcard_error = "FILE REMOVE ERR";
        return false;
    }

    sdcard_error = "OK";
    return true;
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
