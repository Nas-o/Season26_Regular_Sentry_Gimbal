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
    command->lock_requested = (FSI6X_data.CH8 == FS_UP) ? 0U : 1U;

    if (FSI6X_data.CH6 == FS_UP && FSI6X_data.CH7 == FS_MID)
        command->chassis_mode = SPIN_P;
    else if (FSI6X_data.CH6 == FS_UP && FSI6X_data.CH7 == FS_DOWN)
        command->chassis_mode = SPIN_N;
    else
        command->chassis_mode = FLOW_Gimbal;

    const uint8_t aim_enabled =
        (FSI6X_data.CH6 == FS_DOWN && FSI6X_data.CH7 == FS_UP) ? 1U : 0U;
    command->auto_mode = aim_enabled ? CAR : NONE;
    command->shoot_mode = aim_enabled ? READY : CLOSE;

    command->chassis_x = (int16_t)FSI6X_data.CH1;
    command->chassis_y = (int16_t)FSI6X_data.CH2;
    command->chassis_r = (int16_t)FSI6X_data.CH4;
    command->gimbal_yaw = (int16_t)FSI6X_data.CH4;
    command->gimbal_pitch = (int16_t)FSI6X_data.CH3;
    command->trigger = (FSI6X_data.CH5 == FS_DOWN) ? 660 : 0;
    return 1U;
}
