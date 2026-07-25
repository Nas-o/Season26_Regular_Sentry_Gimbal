/**
 * @file  remote_control.cpp
 * @brief 遥控输入源选择、安全仲裁与全局控制指令更新
 * @note  各遥控器在自己的 command 模块中解释物理通道。本文件只负责
 *        在线计时、输入源优先级、离线保护和公共控制状态机。
 */
#include "remote_control.hpp"

#include "Auto_control.hpp"
#include "Chassis.hpp"
#include "DT7.hpp"
#include "FSI6X.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"
#include "Power_switch.hpp"
#include "VT13.hpp"
#include "remote_command.hpp"

#include <string.h>

#define SHOOT_MODE_DEBOUNCE_TICKS 10
#define SINGLE_COMMAND_TICKS      10
#define REMOTE_TASK_PERIOD_MS      2

namespace
{
RemoteCommand active_command;

struct ShootInputState
{
    ShootMode candidate = CLOSE;
    uint8_t stable_ticks = 0;
    uint8_t single_zone_latched = 0;
    uint8_t single_command_ticks = 0;
    RemoteSource source = REMOTE_SOURCE_NONE;
};

ShootInputState shoot_input;

void OnlineTick()
{
    // 在线计数按毫秒递减，收到合法数据帧时由协议模块重新装载。
    DT7_data.online =
        (DT7_data.online > REMOTE_TASK_PERIOD_MS) ?
        DT7_data.online - REMOTE_TASK_PERIOD_MS : 0;
    FSI6X_data.online =
        (FSI6X_data.online > REMOTE_TASK_PERIOD_MS) ?
        FSI6X_data.online - REMOTE_TASK_PERIOD_MS : 0;
    VT13_data.online =
        (VT13_data.online > REMOTE_TASK_PERIOD_MS) ?
        VT13_data.online - REMOTE_TASK_PERIOD_MS : 0;
}

void RemotePick()
{
    // 输入源优先级：DT7 > FSI6X > VT13。
    if (DT7_BuildRemoteCommand(&active_command))
        return;
    if (FSI6X_BuildRemoteCommand(&active_command))
        return;
    if (VT13_BuildRemoteCommand(&active_command))
        return;

    memset(&active_command, 0, sizeof(active_command));
    active_command.lock_requested = 1U;
}

void RCdataSync()
{
    // RC_data 仅为底盘旧接口保留，不再承载设备模式映射。
    if (!active_command.online)
    {
        memset(&RC_data, 0, sizeof(RC_data));
        return;
    }

    RC_data.rc.ch[0] = active_command.chassis_x;
    RC_data.rc.ch[1] = active_command.chassis_y;
    RC_data.rc.ch[2] = active_command.chassis_r;
    RC_data.rc.ch[3] = active_command.gimbal_pitch;
    RC_data.rc.ch[4] = active_command.trigger;
    RC_data.key.v = active_command.key;
    RC_data.mouse.x = active_command.mouse_x;
    RC_data.mouse.y = active_command.mouse_y;
    RC_data.mouse.z = active_command.mouse_z;
    RC_data.mouse.press_l = active_command.mouse_left;
    RC_data.mouse.press_r = active_command.mouse_right;
    RC_data.mouse.press_mid = active_command.mouse_middle;
    RC_data.online = 1;
}

void LockAll()
{
    Global.Control.mode = LOCK;
    Global.Auto.mode = NONE;
    Global.Chassis.mode = FLOW_Chassis;
    Global.Chassis.input.x = 0.0f;
    Global.Chassis.input.y = 0.0f;
    Global.Chassis.input.r = 0.0f;
    Global.Shoot.shoot_mode = CLOSE;
    Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;

    Chassis_SetX(0.0f);
    Chassis_SetY(0.0f);
    memset(&RC_data, 0, sizeof(RC_data));
    shoot_input = ShootInputState{};
}

void ShootUpdate(const RemoteCommand &command)
{
    if (command.source != shoot_input.source)
    {
        shoot_input = ShootInputState{};
        shoot_input.source = command.source;
        Global.Shoot.shoot_mode = CLOSE;
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
    }

    if (command.shoot_mode != shoot_input.candidate)
    {
        shoot_input.candidate = command.shoot_mode;
        shoot_input.stable_ticks = 0;
    }
    else if (shoot_input.stable_ticks < SHOOT_MODE_DEBOUNCE_TICKS)
    {
        shoot_input.stable_ticks++;
    }

    if (shoot_input.stable_ticks >= SHOOT_MODE_DEBOUNCE_TICKS)
        Global.Shoot.shoot_mode = command.shoot_mode;

    if (Global.Shoot.shoot_mode == CLOSE || Global.Auto.mode != NONE)
    {
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
        shoot_input.single_zone_latched = 0;
        shoot_input.single_command_ticks = 0;
        return;
    }

    if (command.trigger >= 300 && command.trigger <= 660)
    {
        Global.Shoot.manual_trigger_mode = ONLY_AIM_HIGH;
        shoot_input.single_zone_latched = 0;
        shoot_input.single_command_ticks = 0;
        return;
    }

    const uint8_t in_single_zone =
        (command.trigger >= -660 && command.trigger <= -330) ? 1U : 0U;
    if (in_single_zone && !shoot_input.single_zone_latched)
    {
        shoot_input.single_zone_latched = 1U;
        shoot_input.single_command_ticks = SINGLE_COMMAND_TICKS;
    }
    else if (!in_single_zone)
    {
        shoot_input.single_zone_latched = 0U;
    }

    if (shoot_input.single_command_ticks > 0)
    {
        Global.Shoot.manual_trigger_mode = SINGLE;
        shoot_input.single_command_ticks--;
    }
    else
    {
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
    }
}
} // namespace

void RC_Controller()
{
    if (!active_command.online || active_command.lock_requested)
    {
        LockAll();
        return;
    }

    Global.Control.mode = RC;
    Global.Chassis.mode = active_command.chassis_mode;
    Global.Auto.mode = active_command.auto_mode;

    Chassis_SetX(active_command.chassis_x / 40.0f);
    Chassis_SetY(active_command.chassis_y / 40.0f);

    // 先更新自瞄模式，再决定本周期是否接受手动云台增量。
    if (!AutoAim_IsControllingGimbal())
    {
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch +
                             active_command.gimbal_pitch / 2000.0f);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw -
                           active_command.gimbal_yaw / 2000.0f);
    }

    if (active_command.power2_control_valid)
    {
        if (active_command.power2_enabled)
            Power_TurnOn(power2);
        else
            Power_TurnOff(power2);
    }

    ShootUpdate(active_command);
}

void Remote_Tasks(void)
{
    OnlineTick();
    RemotePick();
    RCdataSync();
    RC_Controller();
}
