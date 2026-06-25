#ifndef SD2CMT2_MENU_H
#define SD2CMT2_MENU_H

#include <stdbool.h>
#include "../drivers/keypad.h"

typedef enum
{
    MENU_PLAY_MODE_NORMAL = 0,
    MENU_PLAY_MODE_ULTRA_TURBO
} menu_play_mode_t;

/*
    MOTOR: session starts on MOTOR HIGH and follows MOTOR while no manual
    SELECT override has been made.  MANUAL: SELECT alone controls transport.
*/
typedef enum
{
    PLAY_CONTROL_MOTOR = 0,
    PLAY_CONTROL_MANUAL
} play_control_mode_t;

typedef enum
{
    MENU_ACTION_NONE = 0,
    MENU_ACTION_BACK
} menu_action_t;

void menu_init(void);
menu_action_t menu_handle_event(button_event_t event);
void menu_render(void);
menu_play_mode_t menu_get_play_mode(void);
bool menu_get_invert_signal(void);
play_control_mode_t menu_get_play_control_mode(void);
const char *play_control_mode_label(play_control_mode_t mode);

#endif
