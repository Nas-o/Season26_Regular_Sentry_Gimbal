#include "FSI6X.hpp"
#include "remote_command.hpp"

#include <string.h>

uint8_t FSI6X_BuildRemoteCommand(RemoteCommand *command)
{
    if (command == NULL || FSI6X_data.online <= 0 || FSI6X_data.ConnectState == 0)
        return 0U;

    memset(command, 0, sizeof(*command));
    command->source = REMOTE_SOURCE_FSI6X;
    command->online = 1U;

    const uint8_t ch5_up = (FSI6X_data.CH5 == FS_UP) ? 1U : 0U;
    const uint8_t ch5_valid =
        (ch5_up || FSI6X_data.CH5 == FS_DOWN) ? 1U : 0U;
    const uint8_t ch6_up = (FSI6X_data.CH6 == FS_UP) ? 1U : 0U;
    const uint8_t ch6_down = (FSI6X_data.CH6 == FS_DOWN) ? 1U : 0U;
    const uint8_t ch7_up = (FSI6X_data.CH7 == FS_UP) ? 1U : 0U;
    const uint8_t ch7_mid = (FSI6X_data.CH7 == FS_MID) ? 1U : 0U;
    const uint8_t ch7_down = (FSI6X_data.CH7 == FS_DOWN) ? 1U : 0U;
    const uint8_t ch7_valid = (ch7_up || ch7_mid || ch7_down) ? 1U : 0U;
    const uint8_t aim_on = (ch5_up && ch6_up) ? 1U : 0U;

    command->lock_requested = (FSI6X_data.CH8 == FS_UP) ? 0U : 1U;
    command->shoot_mode = ch5_up ? READY : CLOSE;
    command->auto_mode = aim_on ? CAR : NONE;
    command->gimbal_on =
        (aim_on || (ch5_valid && ch6_down)) ? 1U : 0U;
    command->chassis_on =
        (ch7_valid && (!aim_on || ch7_up)) ? 1U : 0U;
    command->trigger = 0;
    if (ch7_up)
    {
        command->chassis_mode = aim_on ? Navigation : SPIN_P;
    }
    else if (aim_on)
    {
        command->chassis_mode = FLOW_Chassis;
    }
    else if (ch7_mid)
    {
        command->chassis_mode = FLOW_Gimbal;
    }
    else
    {
        command->chassis_mode = FLOW_Chassis;
    }
    if (FSI6X_data.CH9 == FS_UP && FSI6X_data.CH10 == FS_DOWN)
        command->trigger = 660;
    else if (FSI6X_data.CH9 == FS_UP && FSI6X_data.CH10 == FS_UP)
        command->trigger = -660;

    command->chassis_x = (int16_t)FSI6X_data.CH1;
    command->chassis_y = (int16_t)FSI6X_data.CH2;
    command->chassis_r = (int16_t)FSI6X_data.CH4;
    command->gimbal_yaw = (int16_t)FSI6X_data.CH4;
    command->gimbal_pitch = (int16_t)FSI6X_data.CH3;
    return 1U;
}
