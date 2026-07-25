#include "DT7.hpp"
#include "remote_command.hpp"

#include <string.h>

uint8_t DT7_BuildRemoteCommand(RemoteCommand *command)
{
    if (command == NULL || DT7_data.online <= 0)
        return 0U;

    memset(command, 0, sizeof(*command));
    command->source = REMOTE_SOURCE_DT7;
    command->online = 1U;

    const uint16_t right_switch = DT7_data.rc.s[0];
    const uint16_t left_switch = DT7_data.rc.s[1];
    const uint8_t switches_valid =
        ((right_switch == RC_SW_UP || right_switch == RC_SW_MID ||
          right_switch == RC_SW_DOWN) &&
         (left_switch == RC_SW_UP || left_switch == RC_SW_MID ||
          left_switch == RC_SW_DOWN)) ? 1U : 0U;
    command->lock_requested =
        (!switches_valid ||
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

    command->chassis_x = DT7_data.rc.ch[0];
    command->chassis_y = DT7_data.rc.ch[1];
    command->chassis_r = DT7_data.rc.ch[2];
    command->gimbal_yaw = DT7_data.rc.ch[2];
    command->gimbal_pitch = DT7_data.rc.ch[3];
    command->trigger = DT7_data.rc.ch[4];
    command->mouse_x = DT7_data.mouse.x;
    command->mouse_y = DT7_data.mouse.y;
    command->mouse_z = DT7_data.mouse.z;
    command->mouse_left = DT7_data.mouse.press_l;
    command->mouse_right = DT7_data.mouse.press_r;
    command->key = DT7_data.key.v;
    return 1U;
}
