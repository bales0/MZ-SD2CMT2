#ifndef SD2CMT2_RECORD_SCREEN_H
#define SD2CMT2_RECORD_SCREEN_H

#include "../drivers/keypad.h"

typedef enum
{
    RECORD_SCREEN_ACTION_NONE = 0,
    RECORD_SCREEN_ACTION_STOP_OR_BACK

} record_screen_action_t;

record_screen_action_t record_screen_handle_event(button_event_t event);

void record_screen_render(void);

#endif
