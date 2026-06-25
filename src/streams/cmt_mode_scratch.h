#ifndef SD2CMT2_CMT_MODE_SCRATCH_H
#define SD2CMT2_CMT_MODE_SCRATCH_H

#include <stdint.h>

/*
   One 512-byte area is never needed by the browser and edge recorder at the
   same time. The browser uses it for directory-selection history; LEP/L16
   RECORD uses it as its second 512-byte SD staging sector.
*/
typedef union
{
    char browser_parent_names[16][32];
    uint8_t edge_record_stage_bytes[512];

} cmt_mode_scratch_t;

extern cmt_mode_scratch_t cmt_mode_scratch;

#endif
