#ifndef REMOTE_COMMAND_HPP_
#define REMOTE_COMMAND_HPP_

#include "Global_status.hpp"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum RemoteSource
{
    REMOTE_SOURCE_NONE = 0,
    REMOTE_SOURCE_DT7,
    REMOTE_SOURCE_FSI6X,
    REMOTE_SOURCE_VT13,
} RemoteSource;

/* Unified operator intent. Axis values retain the existing signed channel
 * scale so this refactor does not change control sensitivity. */
typedef struct RemoteCommand
{
    RemoteSource source;
    uint8_t online;
    uint8_t lock_requested;

    ChassisMode chassis_mode;
    AutoMode auto_mode;
    ShootMode shoot_mode;

    int16_t chassis_x;
    int16_t chassis_y;
    int16_t chassis_r;
    int16_t gimbal_yaw;
    int16_t gimbal_pitch;
    int16_t trigger;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t key;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;

    uint8_t power2_control_valid;
    uint8_t power2_enabled;
} RemoteCommand;

uint8_t DT7_BuildRemoteCommand(RemoteCommand *command);
uint8_t FSI6X_BuildRemoteCommand(RemoteCommand *command);
uint8_t VT13_BuildRemoteCommand(RemoteCommand *command);

#ifdef __cplusplus
}
#endif

#endif
