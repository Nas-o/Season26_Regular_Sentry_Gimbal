#include "BoardLink.hpp"

#include "Auto_control.hpp"
#include "CAN_receive_send.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"

#include "dm_imu.hpp"
#include "motor.hpp"
#include "User_math.hpp"

namespace
{
enum class BoardLinkCanId : uint16_t
{
    ChassisSpeedXY = 0x101,
    ChassisSpeedRYaw = 0x102,
    ChassisPowerLimit = 0x103,
    ChassisMode = 0x104,
    ChassisImuAttitude = 0x105,
    ChassisImuGyro = 0x106,
    GimbalRelativeAngle = 0x107,
    ShootTriggerMode = 0x108,
    RefereeData1 = 0x091,
    RefereeData2 = 0x092,
    RefereeData3 = 0x093,
    SaltationMode = 0x109,
};

constexpr uint8_t kCanFrameLength = 8;
constexpr uint8_t kSchedulePeriodTick = 10;
constexpr uint8_t kFastResendTicks = 5;

struct TxFrame
{
    BoardLinkCanId id {};
    void (*pack)(uint8_t data[kCanFrameLength]) = nullptr;
    uint8_t divider = 1;
    uint8_t phase = 0;
};

struct RxFrame
{
    BoardLinkCanId id {};
    void (*handle)(uint8_t data[kCanFrameLength]) = nullptr;
};

struct SchedulerState
{
    uint8_t tick = 0;
    trigger_mode_e last_trigger_mode = TRIGGER_CLOSE;
    control_mode_e last_control_mode = RC;
    chassis_mode_e last_chassis_mode = FLOW_Chassis;
    auto_mode_e last_auto_mode = NONE;
    uint8_t trigger_mode_fast_resend = 0;
    uint8_t mode_fast_resend = 0;
};

inline uint16_t ToRawId(BoardLinkCanId id)
{
    return static_cast<uint16_t>(id);
}

inline bool MatchId(uint16_t raw_id, BoardLinkCanId id)
{
    return raw_id == ToRawId(id);
}

inline void StoreFloat(float value, uint8_t *bytes)
{
    float_to_bytes(value, bytes);
}

inline void StoreUint8(uint8_t value, uint8_t *bytes)
{
    uint8_to_bytes(value, bytes);
}

inline float LoadFloat(uint8_t *bytes)
{
    return bytes_to_float(bytes);
}

inline uint16_t LoadUint16(uint8_t *bytes)
{
    return bytes_to_uint16(bytes);
}

inline uint8_t LoadUint8(uint8_t *bytes)
{
    return bytes_to_uint8(bytes);
}

float SnapshotRelativeAngle(void)
{
    __disable_irq();
    float angle = relative_angle;
    __enable_irq();
    return angle;
}

float ComputeBigYawSetpoint(void)
{
    const float relative = SnapshotRelativeAngle();
    const float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;

    if (AutoAim_ShouldBigYawFollow())
    {
        return relative + encoder_relative;
    }

    return Global.Gimbal.input.yaw;
}

void Pack_XY_Speed(uint8_t data[kCanFrameLength])
{
    StoreFloat(ChassisLinkMotion.x, &data[0]);
    StoreFloat(ChassisLinkMotion.y, &data[4]);
}

void Pack_R_Yaw(uint8_t data[kCanFrameLength])
{
    Gimbal.big_yaw.big_yaw_location_set = ComputeBigYawSetpoint();
    StoreFloat(ChassisLinkMotion.r, &data[0]);
    StoreFloat(Gimbal.big_yaw.big_yaw_location_set, &data[4]);
}

void Pack_Control_Mode(uint8_t data[kCanFrameLength])
{
    StoreFloat(Global.Control.mode, &data[0]);
    StoreFloat(Global.Chassis.mode, &data[4]);
}

void Pack_IMU_Attitude(uint8_t data[kCanFrameLength])
{
    StoreFloat(imu_chassis.pitch, &data[0]);
    StoreFloat(imu_chassis.yaw_cnt, &data[4]);
}

void Pack_IMU_Gyro(uint8_t data[kCanFrameLength])
{
    StoreFloat(imu_chassis.gyro[0], &data[0]);
    StoreFloat(imu_chassis.gyro[2], &data[4]);
}

void Pack_Saltation_Mode(uint8_t data[kCanFrameLength])
{
    StoreFloat(Navigation_receive_1.yaw_speed, &data[0]);
    StoreUint8(Navigation_receive_1.running_state, &data[4]);
    StoreUint8(fromMINIPC.mode, &data[5]);
    StoreUint8(AA.is_scanning, &data[6]);
    StoreUint8(Navigation_receive_1.region_code, &data[7]);
}

void Pack_TRIGGER_MODE(uint8_t data[kCanFrameLength])
{
    StoreFloat(Global.Shoot.trigger_mode, &data[0]);
    StoreFloat(Global.Auto.mode, &data[4]);
}

void Receive_RefereeData1(uint8_t data[kCanFrameLength])
{
    Referee.current_HP = LoadUint16(&data[0]);
    Referee.maximum_HP = LoadUint16(&data[2]);
    Referee.game_type = LoadUint8(&data[4]);
    Referee.game_progress = LoadUint8(&data[5]);
    Referee.stage_remain_time = LoadUint16(&data[6]);
}

void Receive_RefereeData2(uint8_t data[kCanFrameLength])
{
    Referee.projectile_allowance_17mm = LoadUint16(&data[0]);
    Referee.outpost_HP = LoadUint16(&data[2]);
    Referee.base_HP = LoadUint16(&data[4]);
    Referee.launching_frequency = LoadUint8(&data[6]);
}

void Receive_RefereeData3(uint8_t data[kCanFrameLength])
{
    Referee.rfid_status = LoadFloat(&data[0]);
    Referee.initial_speed = LoadFloat(&data[4]);
}

void Receive_RelativeAngle(uint8_t data[kCanFrameLength])
{
    relative_angle = LoadFloat(&data[0]);
    Gimbal.small_yaw.small_from_big_feedforward = LoadFloat(&data[4]);
    relative_tick = HAL_GetTick();
}

const TxFrame kTxFrames[] = {
    {BoardLinkCanId::ChassisSpeedXY, Pack_XY_Speed, 1, 0},
    {BoardLinkCanId::ChassisSpeedRYaw, Pack_R_Yaw, 1, 0},
    {BoardLinkCanId::ChassisImuAttitude, Pack_IMU_Attitude, 1, 0},
    {BoardLinkCanId::ChassisImuGyro, Pack_IMU_Gyro, 1, 0},
    {BoardLinkCanId::ChassisMode, Pack_Control_Mode, 5, 0},
    {BoardLinkCanId::ShootTriggerMode, Pack_TRIGGER_MODE, 5, 2},
    {BoardLinkCanId::SaltationMode, Pack_Saltation_Mode, 5, 4},
};

const RxFrame kRxFrames[] = {
    {BoardLinkCanId::GimbalRelativeAngle, Receive_RelativeAngle},
    {BoardLinkCanId::RefereeData1, Receive_RefereeData1},
    {BoardLinkCanId::RefereeData2, Receive_RefereeData2},
    {BoardLinkCanId::RefereeData3, Receive_RefereeData3},
};

SchedulerState g_scheduler {};

bool ShouldForceTriggerResend(BoardLinkCanId id)
{
    return id == BoardLinkCanId::ShootTriggerMode && g_scheduler.trigger_mode_fast_resend > 0;
}

bool ShouldForceModeResend(BoardLinkCanId id)
{
    return id == BoardLinkCanId::ChassisMode && g_scheduler.mode_fast_resend > 0;
}

void UpdateFastResendState(void)
{
    if (Global.Shoot.trigger_mode != g_scheduler.last_trigger_mode || Global.Auto.mode != g_scheduler.last_auto_mode)
    {
        g_scheduler.last_trigger_mode = Global.Shoot.trigger_mode;
        g_scheduler.last_auto_mode = Global.Auto.mode;
        g_scheduler.trigger_mode_fast_resend = kFastResendTicks;
    }

    if (Global.Control.mode != g_scheduler.last_control_mode || Global.Chassis.mode != g_scheduler.last_chassis_mode)
    {
        g_scheduler.last_control_mode = Global.Control.mode;
        g_scheduler.last_chassis_mode = Global.Chassis.mode;
        g_scheduler.mode_fast_resend = kFastResendTicks;
    }
}

void ConsumeFastResend(BoardLinkCanId id)
{
    if (id == BoardLinkCanId::ShootTriggerMode && g_scheduler.trigger_mode_fast_resend > 0)
    {
        g_scheduler.trigger_mode_fast_resend--;
    }

    if (id == BoardLinkCanId::ChassisMode && g_scheduler.mode_fast_resend > 0)
    {
        g_scheduler.mode_fast_resend--;
    }
}
}

void BoardLink_Init(void)
{
    g_scheduler = SchedulerState{};
}

void BoardLink_TxStep(void)
{
    UpdateFastResendState();

    uint8_t data[kCanFrameLength];
    for (uint8_t i = 0; i < sizeof(kTxFrames) / sizeof(kTxFrames[0]); i++)
    {
        const TxFrame &frame = kTxFrames[i];
        uint8_t should_send = (g_scheduler.tick % frame.divider == frame.phase);

        if (ShouldForceTriggerResend(frame.id) || ShouldForceModeResend(frame.id))
        {
            should_send = 1;
        }

        if (should_send == 0)
        {
            continue;
        }

        memset(data, 0, sizeof(data));
        frame.pack(data);
        if (Fdcanx_SendData(Get_CanHandle(1), ToRawId(frame.id), data, kCanFrameLength) == 0)
        {
            ConsumeFastResend(frame.id);
        }
    }

    g_scheduler.tick++;
    if (g_scheduler.tick >= kSchedulePeriodTick)
    {
        g_scheduler.tick = 0;
    }
}

uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8])
{
    (void)hfdcan;

    for (uint8_t i = 0; i < sizeof(kRxFrames) / sizeof(kRxFrames[0]); i++)
    {
        if (MatchId(id, kRxFrames[i].id))
        {
            kRxFrames[i].handle(data);
            return BOARD_LINK_RX_HANDLED;
        }
    }

    return BOARD_LINK_RX_UNHANDLED;
}
