#include "mzf_loader.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "../drivers/flash_text.h"
#include "../drivers/mzio.h"
#include "../drivers/sdcard.h"
#include "../streams/wav_sample_stream.h"
#include "mz700_fast3.h"

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
#define MZF_LOADER_MZ800_PREFIX_BYTES 6U
#define MZF_LOADER_MZ800_RELOCATOR_BYTES 14U
#define MZF_LOADER_MZ700_UL_PREFIX_BYTES 7U
#define MZF_LOADER_MZ700_UL_LOW_RAM_MAP_BYTES 2U
#define MZF_LOADER_MZ700_UL_DISPLAY_BYTES 14U
#define MZF_LOADER_MZ700_UL_STATUS_PREFIX_BYTES 8U
#define MZF_LOADER_MZ700_UL_STATUS_END_BYTES 1U
#define MZF_LOADER_MZ700_UL_CLEAR_ADDR 0x09DDU
#define MZF_LOADER_MZ700_UL_CURSOR_ADDR 0x1171U
#define MZF_LOADER_MZ700_LOW_RAM_END 0x1000U
#define MZF_LOADER_HIGH_STACK_SAVE_BYTES 4U
#define MZF_LOADER_HIGH_STACK_SET_BYTES 3U
#define MZF_LOADER_HIGH_STACK_RESTORE_BYTES 4U
#define MZF_LOADER_HIGH_SAVED_SP_BYTES 2U
#define MZF_LOADER_HIGH_STACK_BYTES 16U
#define MZF_LOADER_START_DELAY_MS 100UL
#define MZF_LOADER_HEADER_START_DELAY_MS 1000UL
#define MZF_LOADER_HEADER_READY_DELAY_US 1000U
#define MZF_LOADER_HEADER_INITIAL_TIMEOUT_US 5000000UL
#define MZF_LOADER_TIMEOUT_US 500000UL

static bool is_supported_file_type(uint8_t type)
{
    return (type == 0x01U) || (type == 0x76U);
}

static const uint8_t mz800_header_prolog_P[] PROGMEM =
{
    0x3E, 0x08,             /* ld a,8 */
    0xD3, 0xCE,             /* out (0CEh),a */
    0xCD, 0x3E, 0x07,       /* call 073Eh */
    0x36, 0x01,             /* ld (hl),1 */
    0x97,                   /* sub a */
    0x57,                   /* ld d,a */
    0x5F,                   /* ld e,a */
    0xCD, 0x08, 0x03,       /* call 0308h */
    0xCD, 0xBE, 0x02,       /* call 02BEh */
    0xD3, 0xE2,             /* out (0E2h),a */
    0x1A,                   /* ld a,(de) */
    0xD3, 0xE0,             /* out (0E0h),a */
    0x12,                   /* ld (de),a */
    0x13,                   /* inc de */
    0xCB, 0x62,             /* bit 4,d */
    0x28, 0xF5              /* jr z,self-copy */
};

static const uint8_t mz800_high_low_ram_map_P[] PROGMEM =
{
    0x3E, 0x08,             /* ld a,8 */
    0xD3, 0xE0              /* out (0E0h),a */
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
    0x00,                   /* nop; keep interrupts disabled for entry */
    0xE1,                   /* pop hl */
    0xE9                    /* jp (hl) */
};

/*
   Compact receiver used by the MZ-800 header-only UL.
   Its final byte is JP nn opcode (C3).  In the relocated HIGH version
   the builder copies only sizeof(...)-1 bytes, restores SP, and emits
   its own JP <real EXEC>.
*/
static const uint8_t loader_body_mz800_header_P[] PROGMEM =
{
    0x11, 0x02, 0xE0,       /* ld de,0xE002 */
    0xF3,                   /* di */
    0x1A,                   /* l0: ld a,(de) */
    0xE6, 0x20,             /* and 0x20 */
    0x28, 0xFB,             /* jr z,l0 */
    0x3E, 0x03,             /* ld a,3 */
    0x32, 0x03, 0xE0,       /* ld (0E003h),a */
    0xC5,                   /* l1: push bc */
    0x01, 0x00, 0x04,       /* ld bc,0x0400 */
    0x1A,                   /* l2: ld a,(de) */
    0xCB, 0x67,             /* bit 4,a */
    0x28, 0xFB,             /* jr z,l2 */
    0xE6, 0x20,             /* and 0x20 */
    0xB1,                   /* or c */
    0x07,                   /* rlca */
    0x4F,                   /* ld c,a */
    0x3E, 0x02,             /* ld a,2 */
    0x32, 0x03, 0xE0,       /* ld (0E003h),a */
    0x1A,                   /* l3: ld a,(de) */
    0xCB, 0x67,             /* bit 4,a */
    0x20, 0xFB,             /* jr nz,l3 */
    0xE6, 0x20,             /* and 0x20 */
    0xB1,                   /* or c */
    0x07,                   /* rlca */
    0x4F,                   /* ld c,a */
    0x3E, 0x03,             /* ld a,3 */
    0x32, 0x03, 0xE0,       /* ld (0E003h),a */
    0x10, 0xE0,             /* djnz l2 */
    0x71,                   /* ld (hl),c */
    0xC1,                   /* pop bc */
    0x0B,                   /* dec bc */
    0x23,                   /* inc hl */
    0x79,                   /* ld a,c */
    0xB0,                   /* or b */
    0x20, 0xD4,             /* jr nz,l1 */
    0xC3                    /* jp exec - opcode is replaced by HIGH epilogue */
};

static const uint8_t loader_body_mz800_header_high_exx_P[] PROGMEM =
{
    0x11U, 0x02U, 0xE0U, 0xF3U, 0x1AU, 0xE6U, 0x20U, 0x28U,
    0xFBU, 0x3EU, 0x03U, 0x32U, 0x03U, 0xE0U, 0x01U, 0x00U,
    0x04U, 0x1AU, 0xCBU, 0x67U, 0x28U, 0xFBU, 0xE6U, 0x20U,
    0xB1U, 0x07U, 0x4FU, 0x3EU, 0x02U, 0x32U, 0x03U, 0xE0U,
    0x1AU, 0xCBU, 0x67U, 0x20U, 0xFBU, 0xE6U, 0x20U, 0xB1U,
    0x07U, 0x4FU, 0x3EU, 0x03U, 0x32U, 0x03U, 0xE0U, 0x10U,
    0xE0U, 0x71U, 0x23U, 0xD9U, 0x0BU, 0x79U, 0xB0U, 0xD9U,
    0x20U, 0xD4U, 0xC3U
};

static const uint8_t mz700_ul_status_prefix_P[
    MZF_LOADER_MZ700_UL_STATUS_PREFIX_BYTES] PROGMEM =
{
    'L', 'O', 'A', 'D', 'I', 'N', 'G', ' '
};

static_assert((MZF_LOADER_MZ700_UL_PREFIX_BYTES +
               MZF_LOADER_MZ700_UL_LOW_RAM_MAP_BYTES +
               MZF_LOADER_MZ700_UL_DISPLAY_BYTES +
               sizeof(loader_body_mz800_header_high_exx_P) + 2U +
               MZF_LOADER_MZ700_UL_STATUS_PREFIX_BYTES + 11U +
               MZF_LOADER_MZ700_UL_STATUS_END_BYTES) <=
              MZ700_FAST3_RUNTIME_CAPACITY,
              "MZ700 header-only UL runtime exceeds Description");


typedef struct
{
    bool active;
    bool started;
    bool finished;
    bool initial_wait_used;
    bool mz800_header_high;
    mzf_loader_variant_t variant;
    uint16_t loader_address;
    uint16_t loader_size;
    uint16_t data_length;
    uint16_t data_load_address;
    uint16_t exec_address;
    uint8_t file_type;
    uint8_t original_name[MZ700_FAST3_NAME_BYTES];
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

static bool is_mz700_ul_variant(mzf_loader_variant_t variant)
{
    return (variant == MZF_LOADER_VARIANT_MZ700_UL_LOW) ||
           (variant == MZF_LOADER_VARIANT_MZ700_UL_HIGH);
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

static uint16_t mz800_header_low_size(void)
{
    /* Exact working master LOW header-only loader: 29 + 6 + 59 + 2 = 96 B. */
    return (uint16_t)(sizeof(mz800_header_prolog_P) +
                      MZF_LOADER_MZ800_PREFIX_BYTES +
                      sizeof(loader_body_mz800_header_P) + 2U);
}

static uint16_t mz800_header_high_stage2_size(void)
{
    /*
       MZ800 header-only HIGH stage 2:
         verified MZ800/monitor init          10
         map low RAM                           4
         LD BC,size + EXX (count -> BC')       4
         LD HL,load                            3
         stack-free EXX receiver, no final JP 58
         JP real EXEC                         3
       Total: 82 bytes.

       The 14-byte relocator plus this stage fills the complete 96-byte
       executable area of the 128-byte fake header.
    */
    return 82U;
}

static uint16_t mz800_header_high_size(void)
{
    return (uint16_t)(MZF_LOADER_MZ800_RELOCATOR_BYTES +
                      mz800_header_high_stage2_size());
}

static uint16_t mz800_header_high_runtime_footprint(void)
{
    /* The stack-free HIGH receiver needs no storage beyond stage 2. */
    return mz800_header_high_stage2_size();
}

static bool mz700_ul_needs_low_ram_map(void)
{
    return context.data_load_address < MZF_LOADER_MZ700_LOW_RAM_END;
}

static uint8_t mz700_ul_name_length(void)
{
    uint8_t length = 0U;

    while ((length < MZ700_FAST3_NAME_BYTES) &&
           (context.original_name[length] != 0x00U) &&
           (context.original_name[length] != 0x0DU))
    {
        ++length;
    }
    while ((length != 0U) &&
           (context.original_name[length - 1U] == 0x20U))
    {
        --length;
    }

    /* Keep code plus status within the 104-byte Description workspace.
       Mapping a payload below $1000 consumes two additional bytes. */
    const uint8_t maximum = mz700_ul_needs_low_ram_map() ? 11U : 13U;
    if (length > maximum)
    {
        length = maximum;
    }
    return length;
}

static uint8_t mz700_ul_code_size(void)
{
    uint8_t size = (uint8_t)(MZF_LOADER_MZ700_UL_DISPLAY_BYTES +
                             MZF_LOADER_MZ700_UL_PREFIX_BYTES +
                             sizeof(loader_body_mz800_header_high_exx_P) +
                             2U);

    if (mz700_ul_needs_low_ram_map())
    {
        size = (uint8_t)(size + MZF_LOADER_MZ700_UL_LOW_RAM_MAP_BYTES);
    }
    return size;
}

static uint8_t mz700_ul_runtime_size(void)
{
    return (uint8_t)(mz700_ul_code_size() +
                     MZF_LOADER_MZ700_UL_STATUS_PREFIX_BYTES +
                     mz700_ul_name_length() +
                     MZF_LOADER_MZ700_UL_STATUS_END_BYTES);
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
    if (mode == LOADER_MODE_UL_MZ800)
    {
        return MZF_LOADER_MZ800_HEADER_CAPACITY;
    }

    size = (uint16_t)(MZF_LOADER_PREFIX_BYTES +
                      sizeof(loader_body_P));
    return size;
}

static uint16_t loader_code_size(mzf_loader_variant_t variant, uint16_t size)
{
    return (variant == MZF_LOADER_VARIANT_HIGH) ?
        (uint16_t)(size + sizeof(mz800_high_low_ram_map_P) +
                   MZF_LOADER_HIGH_STACK_SAVE_BYTES +
                   MZF_LOADER_HIGH_STACK_SET_BYTES +
                   MZF_LOADER_HIGH_STACK_RESTORE_BYTES) :
        size;
}

static uint16_t loader_footprint(mzf_loader_variant_t variant, uint16_t size)
{
    return (variant == MZF_LOADER_VARIANT_HIGH) ?
        (uint16_t)(loader_code_size(variant, size) +
                   MZF_LOADER_HIGH_SAVED_SP_BYTES +
                   MZF_LOADER_HIGH_STACK_BYTES) :
        size;
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
    uint16_t footprint = loader_footprint(MZF_LOADER_VARIANT_LOW, size);

    if (!ranges_overlap(MZF_LOADER_LOW_LOAD_ADDR, footprint,
                        data_start, data_length))
    {
        *variant = MZF_LOADER_VARIANT_LOW;
        *address = MZF_LOADER_LOW_LOAD_ADDR;
        return true;
    }

    footprint = loader_footprint(MZF_LOADER_VARIANT_HIGH, size);
    if (!ranges_overlap(MZF_LOADER_HIGH_LOAD_ADDR, footprint,
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

    if (((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) ||
         is_mz700_ul_variant(context.variant) ||
         (context.variant == MZF_LOADER_VARIANT_HIGH)) &&
        !context.initial_wait_used)
    {
        timeout = MZF_LOADER_HEADER_INITIAL_TIMEOUT_US;
    }

    while (write_level() != expected)
    {
        if ((uint32_t)(micros() - start) > timeout)
        {
            set_error_P(PSTR("UL WAIT"));
            return false;
        }
    }

    if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) ||
        is_mz700_ul_variant(context.variant) ||
        (context.variant == MZF_LOADER_VARIANT_HIGH))
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
    memcpy(context.original_name, header + 1U, MZ700_FAST3_NAME_BYTES);
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
    if (mode == LOADER_MODE_UL_MZ700)
    {
        const uint8_t runtime_size = mz700_ul_runtime_size();

        if (!ranges_overlap(MZ700_FAST3_RUNTIME_LOW_ADDR, runtime_size,
                            context.data_load_address, context.data_length) &&
            mz700_fast3_stage_is_encodable(MZ700_FAST3_RUNTIME_LOW_ADDR,
                                           runtime_size))
        {
            context.variant = MZF_LOADER_VARIANT_MZ700_UL_LOW;
            context.loader_address = MZ700_FAST3_RUNTIME_LOW_ADDR;
        }
        else if (!ranges_overlap(MZ700_FAST3_RUNTIME_HIGH_ADDR, runtime_size,
                                 context.data_load_address,
                                 context.data_length) &&
                 mz700_fast3_stage_is_encodable(MZ700_FAST3_RUNTIME_HIGH_ADDR,
                                                runtime_size))
        {
            context.variant = MZF_LOADER_VARIANT_MZ700_UL_HIGH;
            context.loader_address = MZ700_FAST3_RUNTIME_HIGH_ADDR;
        }
        else
        {
            return false;
        }

        context.loader_size = runtime_size;
        context.active = true;
        return true;
    }
    if (mode == LOADER_MODE_MZ700_3X)
    {
        const uint8_t runtime_size =
            mz700_fast3_runtime_size(context.original_name);

        if ((runtime_size == 0U) ||
            (runtime_size > MZ700_FAST3_RUNTIME_CAPACITY))
        {
            return false;
        }

        if (!ranges_overlap(MZ700_FAST3_RUNTIME_LOW_ADDR, runtime_size,
                            context.data_load_address, context.data_length) &&
            mz700_fast3_stage_is_encodable(MZ700_FAST3_RUNTIME_LOW_ADDR,
                                           runtime_size))
        {
            context.variant = MZF_LOADER_VARIANT_MZ700_FAST3_LOW;
            context.loader_address = MZ700_FAST3_RUNTIME_LOW_ADDR;
        }
        else if (!ranges_overlap(MZ700_FAST3_RUNTIME_HIGH_ADDR, runtime_size,
                                 context.data_load_address,
                                 context.data_length) &&
                 mz700_fast3_stage_is_encodable(MZ700_FAST3_RUNTIME_HIGH_ADDR,
                                                runtime_size))
        {
            context.variant = MZF_LOADER_VARIANT_MZ700_FAST3_HIGH;
            context.loader_address = MZ700_FAST3_RUNTIME_HIGH_ADDR;
        }
        else
        {
            /* Safe fallback: caller continues with ordinary 1x playback. */
            return false;
        }

        context.loader_size = runtime_size;
        context.active = true;
        return true;
    }
    else if (mode == LOADER_MODE_UL_MZ800)
    {
        /*
           MZ800 HEADER ONLY AUTO LOW/HIGH:

           LOW is the exact already-working master implementation at $1110.
           Use it whenever the payload does not overwrite its 96-byte code.

           HIGH is selected only when LOW would overlap the payload.
           The 14-byte stage-1 code still starts at $1110, but it relocates
           stage 2 to $C000 before the payload transfer starts.
        */
        const uint16_t low_size = mz800_header_low_size();

        if (!ranges_overlap(MZF_LOADER_MZ800_HEADER_ADDR, low_size,
                            context.data_load_address, context.data_length))
        {
            context.variant = MZF_LOADER_VARIANT_MZ800_HEADER;
            context.mz800_header_high = false;
            context.loader_address = MZF_LOADER_MZ800_HEADER_ADDR;
            context.loader_size = low_size;
        }
        else if (!ranges_overlap(MZF_LOADER_HIGH_LOAD_ADDR,
                                 mz800_header_high_runtime_footprint(),
                                 context.data_load_address,
                                 context.data_length))
        {
            context.variant = MZF_LOADER_VARIANT_MZ800_HEADER;
            context.mz800_header_high = true;
            context.loader_address = MZF_LOADER_MZ800_HEADER_ADDR;
            context.loader_size = mz800_header_high_size();
        }
        else
        {
            return false;
        }

        context.active = true;
        return true;
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

    context.loader_size = loader_code_size(context.variant, size);
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
           !is_tc_variant(context.variant) &&
           !mzf_loader_is_mz700_fast3();
}

bool mzf_loader_is_header_only(void)
{
    return context.active &&
           ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) ||
            is_mz700_ul_variant(context.variant));
}

bool mzf_loader_is_mz800_header_high(void)
{
    return context.active &&
           (context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) &&
           context.mz800_header_high;
}

bool mzf_loader_is_mz700_ul(void)
{
    return context.active && is_mz700_ul_variant(context.variant);
}

bool mzf_loader_is_mz700_ul_high(void)
{
    return context.active &&
           (context.variant == MZF_LOADER_VARIANT_MZ700_UL_HIGH);
}

bool mzf_loader_is_mz700_fast3(void)
{
    return context.active &&
           ((context.variant == MZF_LOADER_VARIANT_MZ700_FAST3_LOW) ||
            (context.variant == MZF_LOADER_VARIANT_MZ700_FAST3_HIGH));
}

bool mzf_loader_is_mz700_fast3_high(void)
{
    return context.active &&
           (context.variant == MZF_LOADER_VARIANT_MZ700_FAST3_HIGH);
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
           (is_ic_variant(context.variant) || is_tc_variant(context.variant) ||
            mzf_loader_is_mz700_fast3());
}

mzf_loader_variant_t mzf_loader_get_variant(void)
{
    return context.variant;
}

uint16_t mzf_loader_get_loader_size(void)
{
    return context.active ? context.loader_size : 0U;
}

bool mzf_loader_patch_loader_header(uint8_t *header)
{
    if ((header == NULL) || !context.active)
    {
        return false;
    }

    if (mzf_loader_is_mz700_fast3())
    {
        return mz700_fast3_build_header(header, context.loader_address,
                                        (uint8_t)context.loader_size,
                                        context.data_length,
                                        context.data_load_address,
                                        context.exec_address,
                                        context.original_name);
    }

    if (mzf_loader_is_mz700_ul())
    {
        uint8_t runtime[MZ700_FAST3_RUNTIME_CAPACITY];
        uint16_t built = mzf_loader_build_loader(
            runtime, MZ700_FAST3_RUNTIME_CAPACITY);

        return (built == context.loader_size) &&
               mz700_header_only_build(header, context.loader_address,
                                       runtime, (uint8_t)built);
    }

    if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) ||
        is_ic_variant(context.variant))
    {
        /*
           SIZE=$0000 is intentional.  On the MZ800 monitor path used here
           it is not treated as an empty payload: the 16-bit relocation
           count wraps and executes the known 65536-byte self-copy before
           control reaches EXEC=$1110.  LOAD=$1200 must therefore remain
           paired with this fake-header scheme.
        */
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
        return true;
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
        return true;
    }

    write_le16(header + MZF_HEADER_DATA_LENGTH_OFFSET, context.loader_size);
    write_le16(header + MZF_HEADER_LOAD_ADDRESS_OFFSET, context.loader_address);
    write_le16(header + MZF_HEADER_EXEC_ADDRESS_OFFSET, context.loader_address);
    return true;
}

uint16_t mzf_loader_build_loader(uint8_t *destination, uint16_t capacity)
{
    uint16_t offset = 0U;
    uint16_t high_saved_sp_address = 0U;

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

    if (is_mz700_ul_variant(context.variant))
    {
        const uint8_t name_length = mz700_ul_name_length();
        const uint16_t status_address =
            (uint16_t)(context.loader_address + mz700_ul_code_size());

        /* QMSGX prints through PRNT3, so a converted $C6 byte is not a
           monitor clear command.  Clear the full character/attribute VRAM
           directly with the verified 1Z-009A fill routine at $09DD.  It
           returns HL=$D800; L is therefore zero and LD H,L cheaply produces
           cursor position $0000 for ($1171).  Keep all ROM calls before a
           possible low-RAM mapping switch. */
        destination[offset++] = 0x21U;           /* ld hl,$D000 */
        write_le16(destination + offset, 0xD000U);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0xCDU;           /* call $09DD */
        write_le16(destination + offset, MZF_LOADER_MZ700_UL_CLEAR_ADDR);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0x65U;           /* ld h,l -> hl=$0000 */
        destination[offset++] = 0x22U;           /* ld ($1171),hl */
        write_le16(destination + offset, MZF_LOADER_MZ700_UL_CURSOR_ADDR);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0x11U;           /* ld de,status */
        write_le16(destination + offset, status_address);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0xDFU;           /* rst $18: QMSGX */

        if (mz700_ul_needs_low_ram_map())
        {
            destination[offset++] = 0xD3U;
            destination[offset++] = 0xE0U;       /* out ($E0),a: low RAM */
        }

        destination[offset++] = 0x01U;           /* ld bc,payload size */
        write_le16(destination + offset, context.data_length);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0xD9U;           /* exx: count -> BC' */

        destination[offset++] = 0x21U;           /* ld hl,payload load */
        write_le16(destination + offset, context.data_load_address);
        offset = (uint16_t)(offset + 2U);

        for (uint8_t i = 0U;
             i < (uint8_t)(sizeof(loader_body_mz800_header_high_exx_P) - 1U);
             ++i)
        {
            destination[offset++] =
                pgm_read_byte(loader_body_mz800_header_high_exx_P + i);
        }

        destination[offset++] = 0xC3U;           /* jp real exec */
        write_le16(destination + offset, context.exec_address);
        offset = (uint16_t)(offset + 2U);

        for (uint8_t i = 0U;
             i < MZF_LOADER_MZ700_UL_STATUS_PREFIX_BYTES; ++i)
        {
            destination[offset++] =
                pgm_read_byte(mz700_ul_status_prefix_P + i);
        }
        memcpy(destination + offset, context.original_name, name_length);
        offset = (uint16_t)(offset + name_length);
        destination[offset++] = 0x0DU;
        return offset;
    }

    if (context.variant == MZF_LOADER_VARIANT_HIGH)
    {
        high_saved_sp_address =
            (uint16_t)(context.loader_address + context.loader_size);

        destination[offset++] = 0xEDU;
        destination[offset++] = 0x73U;
        write_le16(destination + offset, high_saved_sp_address);
        offset = (uint16_t)(offset + 2U);

        destination[offset++] = 0x31U;
        write_le16(destination + offset,
                   (uint16_t)(high_saved_sp_address +
                              MZF_LOADER_HIGH_SAVED_SP_BYTES +
                              MZF_LOADER_HIGH_STACK_BYTES));
        offset = (uint16_t)(offset + 2U);
    }

    if (context.variant == MZF_LOADER_VARIANT_HIGH)
    {
        for (uint8_t i = 0U; i < sizeof(mz800_high_low_ram_map_P); ++i)
        {
            destination[offset++] =
                (uint8_t)pgm_read_byte(mz800_high_low_ram_map_P + i);
        }
    }
    else if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) &&
             !context.mz800_header_high)
    {
        /*
           EXACT working LOW header-only loader from GitHub master.
           This byte-generation path is intentionally untouched.
        */
        for (uint8_t i = 0U; i < sizeof(mz800_header_prolog_P); ++i)
        {
            destination[offset++] =
                (uint8_t)pgm_read_byte(mz800_header_prolog_P + i);
        }

        destination[offset++] = 0x01U;       /* ld bc,payload size */
        write_le16(destination + offset, context.data_length);
        offset = (uint16_t)(offset + 2U);

        destination[offset++] = 0x21U;       /* ld hl,payload load */
        write_le16(destination + offset, context.data_load_address);
        offset = (uint16_t)(offset + 2U);

        for (uint16_t i = 0U; i < sizeof(loader_body_mz800_header_P); ++i)
        {
            destination[offset++] =
                (uint8_t)pgm_read_byte(loader_body_mz800_header_P + i);
        }

        write_le16(destination + offset, context.exec_address);
        offset = (uint16_t)(offset + 2U);
        return offset;
    }
    else if ((context.variant == MZF_LOADER_VARIANT_MZ800_HEADER) &&
             context.mz800_header_high)
    {
        const uint16_t stage2_size = mz800_header_high_stage2_size();
        const uint16_t stage2_source =
            (uint16_t)(MZF_LOADER_MZ800_HEADER_ADDR +
                       MZF_LOADER_MZ800_RELOCATOR_BYTES);
        const uint16_t stage2_address = MZF_LOADER_HIGH_LOAD_ADDR;

        high_saved_sp_address =
            (uint16_t)(stage2_address + stage2_size);

        /* Stage 1 @ $1110: copy ONLY stage 2 to $C000 and jump there. */
        destination[offset++] = 0x21U;       /* ld hl,$111E */
        write_le16(destination + offset, stage2_source);
        offset = (uint16_t)(offset + 2U);

        destination[offset++] = 0x11U;       /* ld de,$C000 */
        write_le16(destination + offset, stage2_address);
        offset = (uint16_t)(offset + 2U);

        destination[offset++] = 0x01U;       /* ld bc,$0052 */
        write_le16(destination + offset, stage2_size);
        offset = (uint16_t)(offset + 2U);

        destination[offset++] = 0xEDU;
        destination[offset++] = 0xB0U;       /* ldir */

        destination[offset++] = 0xC3U;       /* jp $C000 */
        write_le16(destination + offset, stage2_address);
        offset = (uint16_t)(offset + 2U);

        /* Stage 2 image, assembled for execution at $C000. */
        /*
           The fake MZ800 header deliberately uses SIZE=$0000,
           LOAD=$1200 and EXEC=$1110.  The monitor's 16-bit zero-size path
           therefore performs the known 65536-byte self-LDIR at $1200
           before finally entering this loader at $1110.

           After that SIZE=$0000 quirk, the HIGH path needs enough of the
           normal HLL monitor initialization for the memory-mapped
           $E002/$E003 cassette handshake to work.  The smallest sequence
           verified on real hardware so far is:

             LD A,8
             OUT (CE),A
             CALL 073E
             CALL 0308

           Tests showed that removing either CALL $073E or CALL $0308
           breaks the first SENSE/WRITE handshake.  LD (HL),1, zeroing
           A/DE and CALL $02BE are not required for this HIGH path.
           This is the minimum VERIFIED sequence, not a proof that no
           shorter equivalent sequence or direct register setup exists.
        */
        destination[offset++] = 0x3EU;
        destination[offset++] = 0x08U;
        destination[offset++] = 0xD3U;
        destination[offset++] = 0xCEU;
        destination[offset++] = 0xCDU;
        destination[offset++] = 0x3EU;
        destination[offset++] = 0x07U;
        destination[offset++] = 0xCDU;
        destination[offset++] = 0x08U;
        destination[offset++] = 0x03U;

        /* Make low RAM writable before the payload transfer. */
        for (uint8_t i = 0U; i < sizeof(mz800_high_low_ram_map_P); ++i)
        {
            destination[offset++] =
                pgm_read_byte(mz800_high_low_ram_map_P + i);
        }

        /*
           Keep the remaining payload length in BC'.  Main BC is then free
           for the four handshake pairs that assemble each byte.  Because
           the transfer loop uses no PUSH/POP, payloads starting in low RAM
           cannot overwrite a live loader stack.
        */
        destination[offset++] = 0x01U;       /* ld bc,payload size */
        write_le16(destination + offset, context.data_length);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0xD9U;       /* exx -> BC' = size */

        destination[offset++] = 0x21U;       /* ld hl,payload load */
        write_le16(destination + offset, context.data_load_address);
        offset = (uint16_t)(offset + 2U);

        for (uint8_t i = 0U;
             i < (uint8_t)(sizeof(loader_body_mz800_header_high_exx_P) - 1U);
             ++i)
        {
            destination[offset++] =
                pgm_read_byte(loader_body_mz800_header_high_exx_P + i);
        }

        destination[offset++] = 0xC3U;       /* jp real exec */
        write_le16(destination + offset, context.exec_address);
        offset = (uint16_t)(offset + 2U);

        return offset;
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

    if (context.variant == MZF_LOADER_VARIANT_HIGH)
    {
        for (uint16_t i = 0U; i < (uint16_t)(sizeof(loader_body_P) - 1U); ++i)
        {
            destination[offset++] =
                (uint8_t)pgm_read_byte(loader_body_P + i);
        }

        destination[offset++] = 0xEDU;
        destination[offset++] = 0x7BU;
        write_le16(destination + offset, high_saved_sp_address);
        offset = (uint16_t)(offset + 2U);
        destination[offset++] = 0xE9U;
    }
    else
    {
        for (uint16_t i = 0U; i < sizeof(loader_body_P); ++i)
        {
            destination[offset++] =
                (uint8_t)pgm_read_byte(loader_body_P + i);
        }
    }
    return offset;
}

bool mzf_loader_begin(void)
{
    if (!context.active)
    {
        set_error_P(PSTR("UL ARG"));
        return false;
    }

    if (context.started)
    {
        return true;
    }

    if (!sdcard_file_seek(context.data_offset))
    {
        set_error_P(PSTR("UL SEEK"));
        return false;
    }

    context.transferred = 0UL;
    context.finished = false;
    context.initial_wait_used = false;
    context.started = true;

    mz_sense_set_fast(true);
    if (mzf_loader_is_header_only())
    {
        mz_read_set_fast(false);
        delay(MZF_LOADER_HEADER_START_DELAY_MS);
        mz_read_set_fast(true);
        delayMicroseconds(MZF_LOADER_HEADER_READY_DELAY_US);
    }
    else
    {
        mz_read_set_fast(true);
        delay(MZF_LOADER_START_DELAY_MS);
    }
    return true;
}

bool mzf_loader_pump(uint8_t max_bytes)
{
    uint8_t *work;
    uint16_t request;
    int16_t received;

    if (!context.active || !context.started)
    {
        set_error_P(PSTR("UL ARG"));
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
        set_error_P((received < 0) ? PSTR("UL READ") : PSTR("UL SHORT"));
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
