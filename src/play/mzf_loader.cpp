#include "mzf_loader.h"

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
#define MZF_LOADER_MZ800_HEADER_TYPE 0xBBU
#define MZF_LOADER_MZ800_HEADER_OFFSET 0x20U
#define MZF_LOADER_MZ800_HEADER_ADDR 0x1110U
#define MZF_LOADER_MZ800_HEADER_LOAD_ADDR 0x1200U
#define MZF_LOADER_MZ800_HEADER_CAPACITY (128U - MZF_LOADER_MZ800_HEADER_OFFSET)
#define MZF_LOADER_METADATA_OFFSET 0x18U
#define MZF_LOADER_WORKSPACE_RESTORE_BYTES 13U
#define MZF_LOADER_IC_1_4_SPEED_BYTE 0x11U
#define MZF_LOADER_IC_1_3_SPEED_BYTE 0x16U
#define MZF_LOADER_IC_1_2_SPEED_BYTE 0x20U
#define MZF_LOADER_TC_1_3_SPEED_BYTE 0x1BU
#define MZF_LOADER_TC_1_2_SPEED_BYTE 0x29U
#define MZF_LOADER_TC_LOADER_ADDR 0xD400U
#define MZF_LOADER_TC_LOADER_SIZE 90U
#define MZF_LOADER_TC_SPEED_OFFSET 0x4BU
#define MZF_LOADER_TC_FILE_TYPE_OFFSET 0x4CU
#define MZF_LOADER_TC_WORKSPACE_OFFSET 0x4DU
#define MZF_LOADER_TC_HEADER_TAG_OFFSET 0x18U
#define MZF_LOADER_PREFIX_BYTES 10U
#define MZF_LOADER_MZ800_INITIAL_TIMEOUT_US 2000000UL
#define MZF_LOADER_TIMEOUT_US 500000UL

static bool is_supported_file_type(uint8_t type)
{
    return (type == 0x01U) || (type == 0x76U);
}
static const uint8_t mz800_header_prolog_P[] PROGMEM =
{
    0x3E, 0x08,             /* ld a,8 */
    0xD3, 0xCE              /* out (0CEh),a */
};
static const uint8_t ic_header_loader_P[MZF_LOADER_MZ800_HEADER_CAPACITY] PROGMEM =
{
    0x3E, 0x08, 0xD3, 0xCE, 0xCD, 0x3E, 0x07, 0x36,
    0x01, 0x97, 0x57, 0x5F, 0xCD, 0x08, 0x03, 0xCD,
    0xBE, 0x02, 0xD3, 0xE2, 0x1A, 0xD3, 0xE0, 0x12,
    0x13, 0xCB, 0x62, 0x28, 0xF5, 0x3E, 0xC3, 0x32,
    0x1F, 0x06, 0x21, 0x5C, 0x11, 0x22, 0x20, 0x06,
    0x2A, 0x08, 0x11, 0x7D, 0x32, 0x12, 0x05, 0x7C,
    0x32, 0x4B, 0x0A, 0x2A, 0x0A, 0x11, 0x22, 0x02,
    0x11, 0xCD, 0xF8, 0x04, 0x01, 0xCF, 0x06, 0xED,
    0x71, 0xD3, 0xE2, 0xDA, 0xAA, 0xE9, 0x21, 0x0A,
    0x11, 0xC3, 0x08, 0xED, 0xC5, 0x3A, 0x10, 0x11,
    0xEE, 0x0C, 0x32, 0x10, 0x11, 0x01, 0xCF, 0x06,
    0xED, 0x79, 0xC1, 0xC9, 0x31, 0x39, 0x38, 0x37
};

static const uint8_t tc_loader_template_P[MZF_LOADER_TC_LOADER_SIZE] PROGMEM =
{
    0x3E, 0x08, 0xD3, 0xCE, 0xE5, 0x21, 0x00, 0x00,
    0xD3, 0xE4, 0x7E, 0xD3, 0xE0, 0x77, 0x23, 0x7C,
    0xFE, 0x10, 0x20, 0xF4, 0x3A, 0x4B, 0xD4, 0x32,
    0x4B, 0x0A, 0x3A, 0x4C, 0xD4, 0x32, 0x12, 0x05,
    0x21, 0x4D, 0xD4, 0x11, 0x02, 0x11, 0x01, 0x0D,
    0x00, 0xED, 0xB0, 0xE1, 0x7C, 0xFE, 0xD4, 0x28,
    0x12, 0x2A, 0x04, 0x11, 0xD9, 0x21, 0x00, 0x12,
    0x22, 0x04, 0x11, 0xCD, 0x2A, 0x00, 0xD3, 0xE4,
    0xC3, 0x9A, 0xE9, 0xCD, 0x2A, 0x00, 0xD3, 0xE4,
    0xC3, 0x24, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};
static const uint8_t tc_header_tag_P[8] PROGMEM =
{
    0x5B, 0x96, 0xA5, 0x9D, 0x9A, 0xB7, 0x5D, 0x00
};

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
    bool initial_wait_used;
    mzf_loader_variant_t variant;
    uint16_t loader_address;
    uint16_t loader_size;
    uint16_t data_length;
    uint16_t data_load_address;
    uint16_t exec_address;
    uint8_t file_type;
    uint8_t workspace_restore[MZF_LOADER_WORKSPACE_RESTORE_BYTES];
    uint32_t data_offset;
    uint32_t transferred;
    char error_text[17];
} mzf_loader_context_t;

static mzf_loader_context_t context;

static uint16_t read_le16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
}

static void write_le16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)(value >> 8U);
}

static bool is_ic_mode(loader_mode_t mode)
{
    return (mode == LOADER_MODE_IC_1_4) ||
           (mode == LOADER_MODE_IC_1_3) ||
           (mode == LOADER_MODE_IC_1_2);
}

static bool is_tc_mode(loader_mode_t mode)
{
    return (mode == LOADER_MODE_TC_1_3) ||
           (mode == LOADER_MODE_TC_1_2);
}

static bool is_ic_variant(mzf_loader_variant_t variant)
{
    return (variant == MZF_LOADER_VARIANT_IC_1_4) ||
           (variant == MZF_LOADER_VARIANT_IC_1_3) ||
           (variant == MZF_LOADER_VARIANT_IC_1_2);
}

static bool is_tc_variant(mzf_loader_variant_t variant)
{
    return (variant == MZF_LOADER_VARIANT_TC_1_3) ||
           (variant == MZF_LOADER_VARIANT_TC_1_2);
}

static mzf_loader_variant_t mode_to_variant(loader_mode_t mode)
{
    switch (mode)
    {
        case LOADER_MODE_IC_1_4: return MZF_LOADER_VARIANT_IC_1_4;
        case LOADER_MODE_IC_1_3: return MZF_LOADER_VARIANT_IC_1_3;
        case LOADER_MODE_IC_1_2: return MZF_LOADER_VARIANT_IC_1_2;
        case LOADER_MODE_TC_1_3: return MZF_LOADER_VARIANT_TC_1_3;
        case LOADER_MODE_TC_1_2: return MZF_LOADER_VARIANT_TC_1_2;
        default: return MZF_LOADER_VARIANT_NONE;
    }
}

static uint8_t ic_speed_byte(void)
{
    switch (context.variant)
    {
        case MZF_LOADER_VARIANT_IC_1_2: return MZF_LOADER_IC_1_2_SPEED_BYTE;
        case MZF_LOADER_VARIANT_IC_1_3: return MZF_LOADER_IC_1_3_SPEED_BYTE;
        default: return MZF_LOADER_IC_1_4_SPEED_BYTE;
    }
}

static uint8_t tc_speed_byte(void)
{
    return (context.variant == MZF_LOADER_VARIANT_TC_1_2) ?
        MZF_LOADER_TC_1_2_SPEED_BYTE : MZF_LOADER_TC_1_3_SPEED_BYTE;
}

static void patch_tc_workspace_restore(uint8_t *destination)
{
    destination[MZF_LOADER_TC_FILE_TYPE_OFFSET] = context.file_type;
    write_le16(destination + MZF_LOADER_TC_WORKSPACE_OFFSET,
               context.data_length);
    write_le16(destination + MZF_LOADER_TC_WORKSPACE_OFFSET + 2U,
               context.data_load_address);
    write_le16(destination + MZF_LOADER_TC_WORKSPACE_OFFSET + 4U,
               context.exec_address);
    memcpy(destination + MZF_LOADER_TC_WORKSPACE_OFFSET + 6U,
           context.workspace_restore + 6U,
           MZF_LOADER_WORKSPACE_RESTORE_BYTES - 6U);
}

static uint16_t loader_size(loader_mode_t mode)
{
    uint16_t size;

    if (is_ic_mode(mode))
    {
        return MZF_LOADER_MZ800_HEADER_CAPACITY;
    }
    if (is_tc_mode(mode))
    {
        return MZF_LOADER_TC_LOADER_SIZE;
    }

    size = (uint16_t)(MZF_LOADER_PREFIX_BYTES +
                      sizeof(loader_body_P));
    if (mode == LOADER_MODE_UL_MZ800)
    {
        size = (uint16_t)(size + sizeof(mz800_header_prolog_P));
    }
    return size;
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
                          mzf_loader_variant_t *variant,
                          uint16_t *address)
{
    if (!ranges_overlap(MZF_LOADER_LOW_LOAD_ADDR, size,
                        data_start, data_length))
    {
        *variant = MZF_LOADER_VARIANT_LOW;
        *address = MZF_LOADER_LOW_LOAD_ADDR;
        return true;
    }

    if (!ranges_overlap(MZF_LOADER_HIGH_LOAD_ADDR, size,
                        data_start, data_length))
    {
        *variant = MZF_LOADER_VARIANT_HIGH;
        *address = MZF_LOADER_HIGH_LOAD_ADDR;
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
    uint32_t timeout = MZF_LOADER_TIMEOUT_US;

    if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) &&
        !context.initial_wait_used)
    {
        timeout = MZF_LOADER_MZ800_INITIAL_TIMEOUT_US;
    }

    while (write_level() != expected)
    {
        if ((uint32_t)(micros() - start) > timeout)
        {
            set_error_P(PSTR("UF WAIT"));
            return false;
        }
    }

    if (context.variant == MZF_LOADER_VARIANT_MZ800_HEADER)
    {
        context.initial_wait_used = true;
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

void mzf_loader_reset(void)
{
    memset(&context, 0, sizeof(context));
    context.variant = MZF_LOADER_VARIANT_NONE;
    context.error_text[0] = '\0';
}

bool mzf_loader_prepare(file_format_t format,
                           loader_mode_t mode,
                           const uint8_t *header,
                           uint32_t file_size,
                           uint32_t data_offset)
{
    uint16_t size;
    uint32_t data_end;

    mzf_loader_reset();

    if ((mode == LOADER_MODE_OFF) || (format != FILE_FORMAT_MZF) ||
        (header == NULL))
    {
        return false;
    }

    if (!is_supported_file_type(header[MZF_HEADER_FILE_TYPE_OFFSET]))
    {
        return false;
    }

    context.file_type = header[MZF_HEADER_FILE_TYPE_OFFSET];
    context.data_length = read_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET);
    context.data_load_address = read_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET);
    context.exec_address = read_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET);
    memcpy(context.workspace_restore, header + MZF_HEADER_DATA_LENGTH_OFFSET,
           MZF_LOADER_WORKSPACE_RESTORE_BYTES);
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

    size = loader_size(mode);
    if (mode == LOADER_MODE_AUTO)
    {
        return false;
    }
    if (mode == LOADER_MODE_UL_MZ800)
    {
        if ((size > MZF_LOADER_MZ800_HEADER_CAPACITY) ||
            ranges_overlap(MZF_LOADER_MZ800_HEADER_ADDR, size,
                           context.data_load_address, context.data_length))
        {
            return false;
        }
        context.variant = MZF_LOADER_VARIANT_MZ800_HEADER;
        context.loader_address = MZF_LOADER_MZ800_HEADER_ADDR;
    }
    else if (is_ic_mode(mode))
    {
        if ((size > MZF_LOADER_MZ800_HEADER_CAPACITY) ||
            ranges_overlap(MZF_LOADER_MZ800_HEADER_ADDR, size,
                           context.data_load_address, context.data_length))
        {
            return false;
        }
        context.variant = mode_to_variant(mode);
        context.loader_address = MZF_LOADER_MZ800_HEADER_ADDR;
    }
    else if (is_tc_mode(mode))
    {
        if (ranges_overlap(MZF_LOADER_TC_LOADER_ADDR, size,
                           context.data_load_address, context.data_length))
        {
            return false;
        }
        context.variant = mode_to_variant(mode);
        context.loader_address = MZF_LOADER_TC_LOADER_ADDR;
    }
    else if (!select_loader(context.data_load_address, context.data_length,
                            size, &context.variant, &context.loader_address))
    {
        return false;
    }

    context.loader_size = size;
    context.active = true;
    return true;
}

bool mzf_loader_is_active(void)
{
    return context.active;
}

bool mzf_loader_is_ul_active(void)
{
    return context.active && !is_ic_variant(context.variant) &&
           !is_tc_variant(context.variant);
}

bool mzf_loader_is_header_only(void)
{
    return context.active &&
           (context.variant == MZF_LOADER_VARIANT_MZ800_HEADER);
}

bool mzf_loader_is_ic_turbo(void)
{
    return context.active && is_ic_variant(context.variant);
}

bool mzf_loader_is_tc_turbo(void)
{
    return context.active && is_tc_variant(context.variant);
}

bool mzf_loader_is_tape_turbo(void)
{
    return context.active &&
           (is_ic_variant(context.variant) || is_tc_variant(context.variant));
}

mzf_loader_variant_t mzf_loader_get_variant(void)
{
    return context.variant;
}

uint16_t mzf_loader_get_loader_size(void)
{
    return context.active ? context.loader_size : 0U;
}

void mzf_loader_patch_loader_header(uint8_t *header)
{
    if ((header == NULL) || !context.active)
    {
        return;
    }

    if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) ||
        is_ic_variant(context.variant))
    {
        header[MZF_HEADER_FILE_TYPE_OFFSET] = MZF_LOADER_MZ800_HEADER_TYPE;
        write_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET, 0U);
        write_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET,
                   MZF_LOADER_MZ800_HEADER_LOAD_ADDR);
        write_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET,
                   context.loader_address);
        header[MZF_LOADER_METADATA_OFFSET] = context.file_type;
        header[MZF_LOADER_METADATA_OFFSET + 1U] =
            is_ic_variant(context.variant) ?
            ic_speed_byte() :
            (uint8_t)(context.data_length >> 8U);
        memcpy(header + MZF_LOADER_METADATA_OFFSET + 2U,
               context.workspace_restore, 6U);
        (void)mzf_loader_build_loader(header + MZF_LOADER_MZ800_HEADER_OFFSET,
                                         MZF_LOADER_MZ800_HEADER_CAPACITY);
        return;
    }

    if (is_tc_variant(context.variant))
    {
        write_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET, context.loader_size);
        write_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET, context.loader_address);
        write_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET, context.loader_address);
        for (uint8_t i = 0U; i < sizeof(tc_header_tag_P); ++i)
        {
            header[MZF_LOADER_TC_HEADER_TAG_OFFSET + i] =
                (uint8_t)pgm_read_byte(tc_header_tag_P + i);
        }
        return;
    }

    write_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET, context.loader_size);
    write_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET, context.loader_address);
    write_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET, context.loader_address);
}

uint16_t mzf_loader_build_loader(uint8_t *destination, uint16_t capacity)
{
    uint16_t offset = 0U;

    if ((destination == NULL) || !context.active ||
        (capacity < context.loader_size))
    {
        return 0U;
    }

    if (is_ic_variant(context.variant))
    {
        for (uint8_t i = 0U; i < MZF_LOADER_MZ800_HEADER_CAPACITY; ++i)
        {
            destination[i] = (uint8_t)pgm_read_byte(ic_header_loader_P + i);
        }
        return MZF_LOADER_MZ800_HEADER_CAPACITY;
    }

    if (is_tc_variant(context.variant))
    {
        for (uint8_t i = 0U; i < MZF_LOADER_TC_LOADER_SIZE; ++i)
        {
            destination[i] = (uint8_t)pgm_read_byte(tc_loader_template_P + i);
        }
        destination[MZF_LOADER_TC_SPEED_OFFSET] = tc_speed_byte();
        patch_tc_workspace_restore(destination);
        return MZF_LOADER_TC_LOADER_SIZE;
    }

    if (context.variant == MZF_LOADER_VARIANT_MZ800_HEADER)
    {
        for (uint8_t i = 0U; i < sizeof(mz800_header_prolog_P); ++i)
        {
            destination[offset++] = (uint8_t)pgm_read_byte(mz800_header_prolog_P + i);
        }
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

bool mzf_loader_begin(void)
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
    context.initial_wait_used = false;
    context.started = true;

    mz_sense_set_fast(true);
    mz_read_set_fast(true);
    delay(100U);
    return true;
}

bool mzf_loader_pump(uint8_t max_bytes)
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

bool mzf_loader_is_finished(void)
{
    return context.finished;
}

uint8_t mzf_loader_get_progress_percent(void)
{
    uint32_t percent;

    if (!context.active || (context.data_length == 0U))
    {
        return 0U;
    }

    percent = (context.transferred * 100UL) / context.data_length;
    return (percent > 100UL) ? 100U : (uint8_t)percent;
}

const char *mzf_loader_get_error_text(void)
{
    return context.error_text;
}
