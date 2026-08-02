#include "mzf_ultrafast.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "../drivers/flash_text.h"
#include "../drivers/mzio.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"

#define MZF_HEADER_FILE_TYPE_OFFSET 0x00U
#define MZF_HEADER_DATA_LENGTH_OFFSET 0x12U
#define MZF_HEADER_LOAD_ADDRESS_OFFSET 0x14U
#define MZF_HEADER_EXEC_ADDRESS_OFFSET 0x16U
#define MZF_ULTRAFAST_TIMEOUT_US 500000UL

static bool is_supported_file_type(uint8_t type)
{
    return (type == 0x01U) || (type == 0x76U);
}

/*
   Z80 receiver body. The generated prefix loads:
     BC = payload length, HL = payload destination, DE = exec address,
   then PUSH DE and enters this body.
*/
static const uint8_t loader_body_P[] PROGMEM =
{
    0x11, 0x02, 0xE0,       /* ld de,0xE002 */
    0xDD, 0x21, 0x03, 0xE0, /* ld ix,0xE003 */
    0xAF,                   /* xor a */
    0x3D,                   /* delay: dec a */
    0x20, 0xFD,             /* jr nz,delay */
    0xF3,                   /* di */
    0x1A,                   /* l0: ld a,(de) */
    0xE6, 0x20,             /* and 0x20 */
    0x28, 0xFB,             /* jr z,l0 */
    0xDD, 0x36, 0x00, 0x03, /* ld (ix+0),3 */
    0xC5,                   /* l1: push bc */
    0x01, 0x00, 0x04,       /* ld bc,0x0400 */
    0x1A,                   /* l2: ld a,(de) */
    0xCB, 0x67,             /* bit 4,a */
    0x28, 0xFB,             /* jr z,l2 */
    0xE6, 0x20,             /* and 0x20 */
    0xB1,                   /* or c */
    0x07,                   /* rlca */
    0x4F,                   /* ld c,a */
    0xDD, 0x36, 0x00, 0x02, /* ld (ix+0),2 */
    0x1A,                   /* l3: ld a,(de) */
    0xCB, 0x67,             /* bit 4,a */
    0x20, 0xFB,             /* jr nz,l3 */
    0xE6, 0x20,             /* and 0x20 */
    0xB1,                   /* or c */
    0x07,                   /* rlca */
    0x4F,                   /* ld c,a */
    0xDD, 0x36, 0x00, 0x03, /* ld (ix+0),3 */
    0x10, 0xE2,             /* djnz l2 */
    0x71,                   /* ld (hl),c */
    0xC1,                   /* pop bc */
    0x0B,                   /* dec bc */
    0x23,                   /* inc hl */
    0x79,                   /* ld a,c */
    0xB0,                   /* or b */
    0x20, 0xD6,             /* jr nz,l1 */
    0xFB,                   /* ei */
    0xE1,                   /* pop hl */
    0xE9                    /* jp (hl) */
};

typedef struct
{
    bool active;
    bool started;
    bool finished;
    mzf_ultrafast_variant_t variant;
    uint16_t loader_address;
    uint16_t loader_size;
    uint16_t data_length;
    uint16_t data_load_address;
    uint16_t exec_address;
    uint32_t data_offset;
    uint32_t transferred;
    char error_text[17];
} mzf_ultrafast_context_t;

static mzf_ultrafast_context_t context;

static uint16_t read_le16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
}

static void write_le16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)(value >> 8U);
}

static uint16_t loader_size(void)
{
    return (uint16_t)(10U + sizeof(loader_body_P));
}

static bool ranges_overlap(uint32_t left_start, uint32_t left_length,
                           uint32_t right_start, uint32_t right_length)
{
    uint32_t left_end = left_start + left_length;
    uint32_t right_end = right_start + right_length;

    return (left_start < right_end) && (right_start < left_end);
}

static bool select_loader(uint32_t data_start, uint32_t data_length,
                          uint16_t size,
                          mzf_ultrafast_variant_t *variant,
                          uint16_t *address)
{
    if (!ranges_overlap(MZF_ULTRAFAST_LOW_LOAD_ADDR, size,
                        data_start, data_length))
    {
        *variant = MZF_ULTRAFAST_VARIANT_LOW;
        *address = MZF_ULTRAFAST_LOW_LOAD_ADDR;
        return true;
    }

    if (!ranges_overlap(MZF_ULTRAFAST_HIGH_LOAD_ADDR, size,
                        data_start, data_length))
    {
        *variant = MZF_ULTRAFAST_VARIANT_HIGH;
        *address = MZF_ULTRAFAST_HIGH_LOAD_ADDR;
        return true;
    }

    return false;
}

static void set_error_P(PGM_P text)
{
    flash_text_copy(context.error_text, sizeof(context.error_text), text);
}

static inline uint8_t write_level(void)
{
    return (PINJ & _BV(PJ0)) ? 1U : 0U;
}

static inline void read_set(uint8_t level)
{
    if (level != 0U) PORTE |= _BV(PE4);
    else PORTE &= (uint8_t)~_BV(PE4);
}

static inline void sense_set(uint8_t level)
{
    if (level != 0U) PORTD |= _BV(PD3);
    else PORTD &= (uint8_t)~_BV(PD3);
}

static bool wait_write_level(uint8_t expected)
{
    uint32_t start = micros();

    while (write_level() != expected)
    {
        if ((uint32_t)(micros() - start) > MZF_ULTRAFAST_TIMEOUT_US)
        {
            set_error_P(PSTR("UF WAIT"));
            return false;
        }
    }
    return true;
}

static uint8_t rotate_data(uint8_t value)
{
    return (uint8_t)((value << 2U) | (value >> 6U));
}

static bool send_byte(uint8_t value)
{
    uint8_t data = rotate_data((uint8_t)~value);

    for (uint8_t pair = 0U; pair < 4U; ++pair)
    {
        if (!wait_write_level(0U)) return false;
        read_set((data & 0x80U) ? 1U : 0U);
        sense_set(0U);

        if (!wait_write_level(1U)) return false;
        read_set((data & 0x40U) ? 1U : 0U);
        sense_set(1U);

        data <<= 2U;
    }

    return true;
}

void mzf_ultrafast_reset(void)
{
    memset(&context, 0, sizeof(context));
    context.variant = MZF_ULTRAFAST_VARIANT_NONE;
    context.error_text[0] = '\0';
}

bool mzf_ultrafast_prepare(file_format_t format,
                           bool enabled,
                           const uint8_t *header,
                           uint32_t file_size,
                           uint32_t data_offset)
{
    uint16_t size;
    uint32_t data_end;

    mzf_ultrafast_reset();

    if (!enabled || (format != FILE_FORMAT_MZF) || (header == NULL))
    {
        return false;
    }

    if (!is_supported_file_type(header[MZF_HEADER_FILE_TYPE_OFFSET]))
    {
        return false;
    }

    context.data_length = read_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET);
    context.data_load_address = read_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET);
    context.exec_address = read_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET);
    context.data_offset = data_offset;

    if (context.data_length == 0U)
    {
        return false;
    }

    data_end = (uint32_t)context.data_load_address + context.data_length;
    if ((data_end > 0x10000UL) ||
        ((data_offset + (uint32_t)context.data_length) > file_size))
    {
        return false;
    }

    size = loader_size();
    if (!select_loader(context.data_load_address, context.data_length,
                       size, &context.variant, &context.loader_address))
    {
        return false;
    }

    context.loader_size = size;
    context.active = true;
    return true;
}

bool mzf_ultrafast_is_active(void)
{
    return context.active;
}

mzf_ultrafast_variant_t mzf_ultrafast_get_variant(void)
{
    return context.variant;
}

uint16_t mzf_ultrafast_get_loader_size(void)
{
    return context.active ? context.loader_size : 0U;
}

void mzf_ultrafast_patch_loader_header(uint8_t *header)
{
    if ((header == NULL) || !context.active)
    {
        return;
    }

    write_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET, context.loader_size);
    write_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET, context.loader_address);
    write_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET, context.loader_address);
}

uint16_t mzf_ultrafast_build_loader(uint8_t *destination, uint16_t capacity)
{
    uint16_t offset = 0U;

    if ((destination == NULL) || !context.active ||
        (capacity < context.loader_size))
    {
        return 0U;
    }

    destination[offset++] = 0x01U;
    write_le16(destination + offset, context.data_length);
    offset = (uint16_t)(offset + 2U);

    destination[offset++] = 0x21U;
    write_le16(destination + offset, context.data_load_address);
    offset = (uint16_t)(offset + 2U);

    destination[offset++] = 0x11U;
    write_le16(destination + offset, context.exec_address);
    offset = (uint16_t)(offset + 2U);

    destination[offset++] = 0xD5U;

    for (uint16_t i = 0U; i < sizeof(loader_body_P); ++i)
    {
        destination[offset++] = (uint8_t)pgm_read_byte(loader_body_P + i);
    }

    return offset;
}

bool mzf_ultrafast_begin(void)
{
    if (!context.active)
    {
        set_error_P(PSTR("UF ARG"));
        return false;
    }

    if (context.started)
    {
        return true;
    }

    if (!sdcard_file_seek(context.data_offset))
    {
        set_error_P(PSTR("UF SEEK"));
        return false;
    }

    context.transferred = 0UL;
    context.finished = false;
    context.started = true;

    mz_sense_set_fast(true);
    mz_read_set_fast(true);
    delay(100U);
    return true;
}

bool mzf_ultrafast_pump(uint8_t max_bytes)
{
    uint8_t *work;
    uint16_t request;
    int16_t received;

    if (!context.active || !context.started)
    {
        set_error_P(PSTR("UF ARG"));
        return false;
    }

    if (context.finished)
    {
        return true;
    }

    if (max_bytes == 0U)
    {
        max_bytes = 1U;
    }

    request = (uint16_t)((uint32_t)context.data_length - context.transferred);
    if (request > max_bytes) request = max_bytes;
    if (request > WAV_SAMPLE_STREAM_REFILL_BLOCK)
    {
        request = WAV_SAMPLE_STREAM_REFILL_BLOCK;
    }

    work = wav_sample_stream_get_shared_work_buffer();
    received = sdcard_file_read(work, request);
    if (received != (int16_t)request)
    {
        set_error_P((received < 0) ? PSTR("UF READ") : PSTR("UF SHORT"));
        return false;
    }

    for (uint16_t i = 0U; i < request; ++i)
    {
        if (!send_byte(work[i]))
        {
            return false;
        }
        context.transferred++;
    }

    if (context.transferred >= context.data_length)
    {
        if (!wait_write_level(0U)) return false;
        context.finished = true;
        mz_read_set_fast(false);
        mz_sense_set_fast(true);
    }

    return true;
}

bool mzf_ultrafast_is_finished(void)
{
    return context.finished;
}

uint8_t mzf_ultrafast_get_progress_percent(void)
{
    uint32_t percent;

    if (!context.active || (context.data_length == 0U))
    {
        return 0U;
    }

    percent = (context.transferred * 100UL) / context.data_length;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

const char *mzf_ultrafast_get_error_text(void)
{
    return context.error_text;
}
