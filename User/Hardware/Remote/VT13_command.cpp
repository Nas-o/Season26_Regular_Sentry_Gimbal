#include "DT7.hpp"
#include "VT13.hpp"
#include "remote_command.hpp"

#include <string.h>

uint8_t VT13_BuildRemoteCommand(RemoteCommand *command)
{
    if (command == NULL || VT13_data.online <= 0)
        return 0U;

    memset(command, 0, sizeof(*command));
    command->source = REMOTE_SOURCE_VT13;
    command->online = 1U;
    command->chassis_on = 1U;
    command->gimbal_on = 1U;

    uint16_t right_switch = RC_SW_MID;
    uint16_t left_switch = RC_SW_MID;
    const uint8_t mode_switch_valid = (VT13_data.rc.mode_sw <= 2) ? 1U : 0U;
    if (VT13_data.rc.mode_sw == 0)
        right_switch = RC_SW_DOWN;
    else if (VT13_data.rc.mode_sw == 2)
        right_switch = RC_SW_UP;

    if (VT13_data.rc.mode_sw == 2 || VT13_data.rc.wheel >= 330)
        left_switch = RC_SW_UP;
    else if (VT13_data.rc.wheel <= -330)
        left_switch = RC_SW_DOWN;

    command->lock_requested =
        (!mode_switch_valid || VT13_data.rc.stop != 0 ||
         (right_switch == RC_SW_DOWN && left_switch == RC_SW_DOWN)) ? 1U : 0U;

    if (right_switch == RC_SW_UP && left_switch == RC_SW_MID)
        command->chassis_mode = SPIN_P;
    else if (right_switch == RC_SW_MID && left_switch == RC_SW_DOWN)
        command->chassis_mode = SPIN_N;
    else if (right_switch == RC_SW_UP && left_switch == RC_SW_UP)
        command->chassis_mode = Navigation;
    else if (right_switch == RC_SW_MID && left_switch == RC_SW_MID)
        command->chassis_mode = FLOW_Gimbal;
    else
        command->chassis_mode = FLOW_Chassis;

    command->auto_mode =
        (left_switch == RC_SW_UP && right_switch != RC_SW_DOWN) ? CAR : NONE;
    command->shoot_mode = (left_switch == RC_SW_UP) ? READY : CLOSE;
    command->chassis_x = VT13_data.rc.ch[0];
    command->chassis_y = VT13_data.rc.ch[1];
    command->chassis_r = VT13_data.rc.ch[3];
    command->gimbal_yaw = VT13_data.rc.ch[3];
    command->gimbal_pitch = VT13_data.rc.ch[2];
    command->trigger = VT13_data.rc.shutter ? 660 : 0;
    command->mouse_x = VT13_data.mouse.x;
    command->mouse_y = VT13_data.mouse.y;
    command->mouse_z = VT13_data.mouse.z;
    command->mouse_left = VT13_data.mouse.press_l;
    command->mouse_right = VT13_data.mouse.press_r;
    command->mouse_middle = VT13_data.mouse.middle;
    command->key = VT13_data.key.v;
    command->power2_control_valid = 1U;
    command->power2_enabled = (VT13_data.rc.left_button == 0) ? 1U : 0U;
    return 1U;
}
