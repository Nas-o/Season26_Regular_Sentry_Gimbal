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
        RefereeData1 = 0x091,
        RefereeData2 = 0x092,
        RefereeData3 = 0x093,
        ChassisSpeedXY = 0x101,
        ChassisSpeedRYaw = 0x102,
        ChassisPowerLimit = 0x103,
        ChassisMode = 0x104,
        ChassisImuAttitude = 0x105,
        ChassisImuGyro = 0x106,
        GimbalRelativeAngle = 0x107,
        ShootTriggerMode = 0x108,
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
        Gimbal.big_yaw.pos_set = 0;//ComputeBigYawSetpoint();
        StoreFloat(ChassisLinkMotion.r, &data[0]);
        StoreFloat(Gimbal.big_yaw.pos_set, &data[4]);
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
        StoreUint8(AutoAim.is_scanning, &data[6]);
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
        Gimbal.yaw.big_ff = LoadFloat(&data[4]);
        relative_tick = HAL_GetTick();
    }

    const TxFrame kTxFrames[] = 
    {
        {BoardLinkCanId::ChassisSpeedXY, Pack_XY_Speed, 1, 0},
        {BoardLinkCanId::ChassisSpeedRYaw, Pack_R_Yaw, 1, 0},
        {BoardLinkCanId::ChassisImuAttitude, Pack_IMU_Attitude, 1, 0},
        {BoardLinkCanId::ChassisImuGyro, Pack_IMU_Gyro, 1, 0},
        {BoardLinkCanId::ChassisMode, Pack_Control_Mode, 5, 0},
        {BoardLinkCanId::ShootTriggerMode, Pack_TRIGGER_MODE, 5, 2},
        {BoardLinkCanId::SaltationMode, Pack_Saltation_Mode, 5, 4},
    };

    constexpr uint8_t kTxFrameCount = sizeof(kTxFrames) / sizeof(kTxFrames[0]);

    const RxFrame kRxFrames[] = 
    {
        {BoardLinkCanId::GimbalRelativeAngle, Receive_RelativeAngle},
        {BoardLinkCanId::RefereeData1, Receive_RefereeData1},
        {BoardLinkCanId::RefereeData2, Receive_RefereeData2},
        {BoardLinkCanId::RefereeData3, Receive_RefereeData3},
    };

    /* ============== 通用快发机制 ============== */

    uint8_t g_tick = 0;                        // 调度tick计数
    uint8_t g_fast_resend[kTxFrameCount] {};  // 每帧快发剩余拍数

    /**
     * @brief 根据 CAN ID 查找 TxFrame 索引
     * @return 索引值, 未找到返回 -1
     */
    int8_t FindTxIndex(uint16_t can_id)
    {
        for (uint8_t i = 0; i < kTxFrameCount; i++)
        {
            if (ToRawId(kTxFrames[i].id) == can_id)
                return (int8_t)i;
        }
        return -1;
    }

    /**
     * @brief 内置边沿检测: 监视离散量, 变化时自动触发快发
     * @note  每个 EdgeWatch 关联一个 CAN 帧 + 一个 uint32 值
     *        值变化时自动调用快发, 无需外部介入
     */
    struct EdgeWatch
    {
        uint16_t can_id;
        uint32_t (*sample)(void);  // 采样函数: 返回当前值的压缩表示
        uint32_t last_value;
    };

    // 采样函数: 将关联变量打包为 uint32 以便比较
    uint32_t SampleTriggerMode(void)
    {
        return ((uint32_t)Global.Shoot.trigger_mode << 16) | (uint32_t)Global.Auto.mode;
    }

    uint32_t SampleControlMode(void)
    {
        return ((uint32_t)Global.Control.mode << 16) | (uint32_t)Global.Chassis.mode;
    }

    // 边沿监视表: 需要自动快发的变量在此注册
    EdgeWatch g_edges[] = 
    {
        {BL_ID_TRIGGER_MODE, SampleTriggerMode, 0},
        {BL_ID_CONTROL_MODE, SampleControlMode, 0},
    };

    constexpr uint8_t kEdgeCount = sizeof(g_edges) / sizeof(g_edges[0]);

    /**
     * @brief 扫描所有边沿监视, 检测到变化则触发快发
     */
    void DetectEdges(void)
    {
        for (uint8_t i = 0; i < kEdgeCount; i++)
        {
            uint32_t val = g_edges[i].sample();
            if (val != g_edges[i].last_value)
            {
                g_edges[i].last_value = val;
                int8_t idx = FindTxIndex(g_edges[i].can_id);
                if (idx >= 0)
                    g_fast_resend[idx] = kFastResendTicks;
            }
        }
    }

} // namespace

/* ============== 公共接口 ============== */

void BoardLink_Init(void)
{
    g_tick = 0;
    for (uint8_t i = 0; i < kTxFrameCount; i++)
        g_fast_resend[i] = 0;
    // 初始化边沿监视的 last_value
    for (uint8_t i = 0; i < kEdgeCount; i++)
        g_edges[i].last_value = g_edges[i].sample();
}

void BoardLink_RequestFastResend(uint16_t can_id)
{
    int8_t idx = FindTxIndex(can_id);
    if (idx >= 0)
        g_fast_resend[idx] = kFastResendTicks;
}

void BoardLink_TxStep(void)
{
    // 内置边沿检测
    DetectEdges();

    uint8_t data[kCanFrameLength];
    for (uint8_t i = 0; i < kTxFrameCount; i++)
    {
        const TxFrame &frame = kTxFrames[i];
        bool should_send = (g_tick % frame.divider == frame.phase);

        // 快发覆盖: 有剩余拍数则强制发送
        if (g_fast_resend[i] > 0)
            should_send = true;

        if (!should_send)
            continue;

        memset(data, 0, sizeof(data));
        frame.pack(data);
        if (Fdcanx_SendData(Get_CanHandle(1), ToRawId(frame.id), data, kCanFrameLength) == 0)
        {
            // 发送成功才消耗快发计数
            if (g_fast_resend[i] > 0)
                g_fast_resend[i]--;
        }
    }

    g_tick++;
    if (g_tick >= kSchedulePeriodTick)
        g_tick = 0;
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
