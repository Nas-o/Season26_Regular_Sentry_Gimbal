/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-07 17:16:45
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Auto_control.c
 */

#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "dm_imu.h"
#include "referee_system.h"
#include "UART_data_txrx.h"
#include "USB_VirCom.h"
#include "hipnuc_dec.h"
#include "pid.h"
#include "User_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "CRC8_CRC16.h"
#include "string.h"
#include <math.h>

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;
STM32ROS_data_t stm32send_1;
Sentry_cmd_t Sentry_cmd_1;
Referee_data_t Referee;

/* ========== 唯一自瞄全局状态 ========== */
AutoAim_t AA;

uint32_t aa_rx_timeout_ms    = 100;   // 链路新鲜度: 超时则不信任任何帧 
float aa_yaw_reject_deg      = 60.0f; // 执行安全: 目标yaw与IMU差值超过此阈值 → 拒绝该帧
float aa_yaw_ff_max_deg_s    = 80.0f; // 执行安全: yaw前馈速度限幅 (deg/s)
volatile uint32_t relative_timeout_ms = 8;
volatile uint32_t relative_tick = 0;
volatile uint32_t relative_lost = 0;
volatile uint8_t  relative_seq = 0;

uint8_t RelativeAngle_IsFresh(void)
{
    if (relative_tick == 0)
    {
        return 0;
    }

    return (HAL_GetTick() - relative_tick <= relative_timeout_ms);
}
float aa_scan_yaw_limit_deg  = 60.0f; // 扫描yaw幅度 (deg)
float aa_scan_yaw_speed_deg_s = SCAN_YAW_SPEED; // 扫描yaw速度 (deg/s)
float aa_scan_yaw_step_deg   = 0.16f; // 扫描yaw位置步长 (deg/tick)
float aa_scan_pitch_step_deg = 0.15f; // 扫描pitch步长 (deg/tick)

uint8_t Debug_Sentry_Revive_State = 0;
uint32_t Debug_Sentry_Tx_Count = 0;

/* ---- 原子快照: ISR写shadow，控制任务读 working copy ---- */
volatile MINIPC_data_t  fromMINIPC_shadow;   // ISR写入
volatile uint32_t       minipc_last_rx_tick;  // ISR写入: HAL_GetTick()
volatile uint32_t       minipc_rx_seq = 0;   // ISR写入: 每收一包自增

#define MINIPC_TIMEOUT_MS  100  // 超时门限

/**
 * @brief 原子拷贝 shadow -> working copy
 */
void MINIPC_SyncShadowToWorkingCopy(void)
{
    fromMINIPC = *(MINIPC_data_t *)&fromMINIPC_shadow;
}

uint8_t data[128];
uint8_t rx_data[100];


void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

void decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len)
{
    if (len != 60 || buff[0] != 0xAA) {
        return;
    }

    uint16_t received_crc = (uint16_t)buff[58] | ((uint16_t)buff[59] << 8);

    uint16_t calculated_crc = Get_Modbus_CRC16(buff, 58);

    if (calculated_crc == received_crc) 
    {
        memcpy(target, buff, sizeof(Navigation_data_t));
        Navigation_online = 1;
    }
}

static void Receive_RefereeData1(uint8_t data[8])
{
    Referee.current_HP = bytes_to_uint16(&data[0]);
    Referee.maximum_HP = bytes_to_uint16(&data[2]);
    Referee.game_type = bytes_to_uint8(&data[4]);
    Referee.game_progress = bytes_to_uint8(&data[5]);
    Referee.stage_remain_time = bytes_to_uint16(&data[6]);
}

static void Receive_RefereeData2(uint8_t data[8])
{
    Referee.projectile_allowance_17mm = bytes_to_uint16(&data[0]);
    Referee.outpost_HP = bytes_to_uint16(&data[2]);
    Referee.base_HP = bytes_to_uint16(&data[4]);
    Referee.launching_frequency = bytes_to_uint8(&data[6]);
}

static void Receive_RefereeData3(uint8_t data[8])
{
    Referee.rfid_status = bytes_to_float(&data[0]);
    Referee.initial_speed = bytes_to_float(&data[4]);
}

static void Receive_RelativeAngle(uint8_t data[8])
{
    uint32_t last_tick = relative_tick;
    uint8_t use_seq = (data[5] == 0xA5u && data[6] == 0x5Au);

    relative_angle = bytes_to_float(&data[0]);
    relative_tick = HAL_GetTick();

    if (use_seq)
    {
        uint8_t new_seq = bytes_to_uint8(&data[4]);

        if (last_tick != 0)
        {
            uint8_t step = (uint8_t)(new_seq - relative_seq);
            if (step > 1)
            {
                relative_lost += (uint32_t)(step - 1);
            }
        }

        relative_seq = new_seq;
    }
}

static const CanRxEntry_t GimbalRxTable[] = {
    { CAN_ID_GIMBAL_RELATIVE_ANGLE,     Receive_RelativeAngle  },
    { CAN_ID_REFEREE_DATA_1,            Receive_RefereeData1   },
    { CAN_ID_REFEREE_DATA_2,            Receive_RefereeData2   },
    { CAN_ID_REFEREE_DATA_3,            Receive_RefereeData3   },

};

uint8_t Gimbal_CAN_Dispatch(uint16_t id, uint8_t data[8])
{
    for (uint8_t i = 0; i < sizeof(GimbalRxTable)/sizeof(GimbalRxTable[0]); i++) 
    {
        if (GimbalRxTable[i].id == id) 
        {
            GimbalRxTable[i].handler(data);
            return 1;  // 匹配成功
        }
    }
    return 0;  // 未匹配
}
/**
 * @brief 向导航发送裁判系统数据
 *
 */
void Navigation_Send_Message()
    {
    stm32send_1.remain_hp = Referee.current_HP;                                // 机器人实时血量
    stm32send_1.max_hp = Referee.maximum_HP;                                   // 最大血量
    stm32send_1.game_type = Referee.game_type;                                                      // 比赛类型
    stm32send_1.game_progress = Referee.game_progress;                          // 比赛进程
    stm32send_1.stage_remain_time = Referee.stage_remain_time;                  // 当前阶段剩余时间
    stm32send_1.bullet_remaining_num_17mm = Referee.projectile_allowance_17mm; // 允许发弹量
    stm32send_1.outpost_hp = Referee.outpost_HP;                               // 前哨站血量
    stm32send_1.base_hp = Referee.base_HP;                                     // 基地血量
    stm32send_1.rfid_status = Referee.rfid_status;                               // RFID状态

    static uint8_t buff[sizeof(STM32ROS_data_t) + 3];
    buff[0] = 0xA5;
    memcpy(&buff[1], &stm32send_1, sizeof(STM32ROS_data_t));
    append_CRC16_check_sum(buff, sizeof(buff));

    /* Vircom_Send(buff, sizeof(buff)); */
    HAL_UART_Transmit_DMA(UART7_data.huart, buff, sizeof(buff));
    
}



#if(AUTO_CSU == 1)

/**
 * @brief 向上位机发送自瞄相关数据
 * 
 * @param yaw yaw轴当前角度（弧度）
 * @param pitch pitch轴当前角度（弧度）
 * @param omega yaw轴当前角速度（rad/s）
 */
 void STM32_to_MINIPC()
{
    toMINIPC.header = 0xff;
    toMINIPC.ender = 0x0d;
    toMINIPC.mode = 0;
    toMINIPC.roll = imu.roll / RAD_TO_DEG;
    toMINIPC.pitch = imu.pitch / RAD_TO_DEG;
	toMINIPC.yaw = imu.yaw / RAD_TO_DEG;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    Global.Auto.input.shoot_pitch = fromMINIPC.pitch;
	Global.Auto.input.shoot_yaw = fromMINIPC.yaw;
	Global.Auto.input.fire = fromMINIPC.fire;
} 

#endif

#if(AUTO_TJU == 1)

void STM32_to_MINIPC(float yaw,float pitch,float omega)
{
    toMINIPC.FrameHeader.sof = 0xA5;
    toMINIPC.FrameHeader.crc8 = 0x00;
    toMINIPC.To_minipc_data.curr_pitch = pitch;//IMU_data.AHRS.pitch;
    toMINIPC.To_minipc_data.curr_yaw = yaw;//IMU_data.AHRS.yaw;
    toMINIPC.To_minipc_data.curr_omega = omega;//cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[1];
    toMINIPC.To_minipc_data.autoaim = 1;
    if (Referee_data.robot_id >= 100)
        toMINIPC.To_minipc_data.enemy_color = 1;
    else
        toMINIPC.To_minipc_data.enemy_color = 0;
    toMINIPC.To_minipc_data.state = 0;
    toMINIPC.FrameTailer.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.FrameHeader.sof, 17, 0xffff);
    toMINIPC.enter = 0x0A;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    // VirCom_send(data, sizeof(STM32_data_t));
    UART_SendData(UART1_data, data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    if (fabs(fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw) > PI / 2.0f) // 过零点处理
    {
        if (fromMINIPC.from_minipc_data.shoot_yaw > PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw -= 2 * PI;
        else if (fromMINIPC.from_minipc_data.shoot_yaw < -PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw += 2 * PI;
    }
    Global.Auto.input.shoot_pitch = fromMINIPC.from_minipc_data.shoot_pitch - IMU_data.AHRS.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw;
    Global.Auto.input.fire = fromMINIPC.from_minipc_data.fire;
    Global.Auto.input.target_id = fromMINIPC.from_minipc_data.target_id;
    if(fromMINIPC.from_minipc_data.shoot_pitch==0&&fromMINIPC.from_minipc_data.shoot_yaw==0)
        Global.Auto.input.fire = -1;
}


#endif

#if(AUTO_TongJi == 1)

void STM32_to_MINIPC()
{
    toMINIPC.header[0] = 'S';
    toMINIPC.header[1] = 'P';
    toMINIPC.mode = MODE_SMALL_BUFF;
    toMINIPC.yaw = degree2rad(imu_gimbal.yaw);//IMU_data.AHRS.yaw;
    toMINIPC.pitch = degree2rad(imu_gimbal.pitch);//IMU_data.AHRS.pitch;
    toMINIPC.yaw_vel = imu_gimbal.gyro[2];//IMU_data.gyro[2];
    toMINIPC.pitch_vel = imu_gimbal.gyro[1];//IMU_data.gyro[0];
    toMINIPC.q[0] = imu_gimbal.q[0];//IMU_data.AHRS.q[0];
    toMINIPC.q[1] = imu_gimbal.q[1];//IMU_data.AHRS.q[1];
    toMINIPC.q[2] = imu_gimbal.q[2];//IMU_data.AHRS.q[2];
    toMINIPC.q[3] = imu_gimbal.q[3];//IMU_data.AHRS.q[3];
    toMINIPC.bullet_speed = Referee.initial_speed;
    toMINIPC.bullet_count = Referee.launching_frequency;
    int len = (uint8_t *)&toMINIPC.crc16 - (uint8_t *)&toMINIPC;
    // 此时调用 CRC，无论结构体怎么变都永远正确
    toMINIPC.crc16 = get_CRC16_check_sum(&toMINIPC.header, len, 0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

/**
 * @brief 角度差归一化到 [-180, 180]
 * 
 * @param x 
 * @return float 
 */
static float WrapDeltaDeg(float x)
{
    while (x >  180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

/**
 * @brief 单帧数据合理性检查
 * 
 * @param pkt 
 * @return uint8_t 
 */
static uint8_t AimPacket_IsSane(const MINIPC_data_t *pkt)
{
    float yaw_deg      = rad2degree(pkt->yaw);
    float dyaw_deg     = WrapDeltaDeg(yaw_deg - imu_gimbal.yaw);
    float yaw_ff_deg_s = rad2degree(pkt->yaw_vel);

    if (fabsf(dyaw_deg) > aa_yaw_reject_deg)
        return 0;
    if (fabsf(yaw_ff_deg_s) > aa_yaw_ff_max_deg_s)
        return 0;
    return 1;
}

void MINIPC_to_STM32(void)
{
    /* ---- 链路新鲜度 ---- */
    uint32_t age = HAL_GetTick() - minipc_last_rx_tick;
    AA.rx_fresh = (minipc_last_rx_tick != 0 && age <= aa_rx_timeout_ms) ? 1 : 0;

    /* ---- 检测新包 ---- */
    static uint32_t last_rx_seq = 0;
    uint8_t is_new_pkt = (minipc_rx_seq != last_rx_seq);
    last_rx_seq = minipc_rx_seq;

    /* ---- 语义决策直接取 mode ---- */
    AA.mode_raw = fromMINIPC.mode;
    AA.control_req = AA.rx_fresh && (AA.mode_raw != CTRL_NO_CONTROL);

    /* ---- 只在 fresh && control && 新包 && 数据合理 时更新目标数据 ---- */
    if (is_new_pkt && AA.control_req && AimPacket_IsSane(&fromMINIPC))
    {
        Global.Auto.input.shoot_yaw    = fromMINIPC.yaw;
        Global.Auto.input.shoot_pitch  = fromMINIPC.pitch;
        Global.Auto.input.yaw_ff       = fromMINIPC.yaw_vel;
        Global.Auto.input.pitch_ff     = fromMINIPC.pitch_vel;
        Global.Auto.input.yaw_acc_ff   = fromMINIPC.yaw_acc;
        Global.Auto.input.pitch_acc_ff = fromMINIPC.pitch_acc;
    }


    /* ---- 丢控制时清前馈 ---- */
    if (!AA.control_req) 
    {
        Global.Auto.input.yaw_ff       = 0.0f;
        Global.Auto.input.pitch_ff     = 0.0f;
        Global.Auto.input.yaw_acc_ff   = 0.0f;
        Global.Auto.input.pitch_acc_ff = 0.0f;
    }
}
#endif

/**
 * @brief 跟踪
 * 
 */
static void Auto_Control(void)
{
    float target_yaw_deg = rad2degree(Global.Auto.input.shoot_yaw);
    float current_yaw_deg = imu_gimbal.yaw;
    float delta_yaw = target_yaw_deg - current_yaw_deg;
    if (delta_yaw > 180.0f)  delta_yaw -= 360.0f;
    if (delta_yaw < -180.0f) delta_yaw += 360.0f;
    float target_yaw_cnt = imu_gimbal.yaw_cnt + delta_yaw;

    float target_pitch_deg = rad2degree(Global.Auto.input.shoot_pitch);

    Gimbal_SetYawAngle(target_yaw_cnt);
    Gimbal_SetPitchAngle(-target_pitch_deg);

    AA.target_yaw = target_yaw_cnt;
    AA.target_pitch = -target_pitch_deg;
}


/**
 * @brief 扫描---入口切换
 *
 */
static void Scan_Internal(void)
{
    const float PITCH_MAX = PITCHI_MAX_ANGLE;
    const float PITCH_MIN = PITCHI_MIN_ANGLE;
    float small_yaw_relative_now = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    /* 初始同步 */
    if (!AA.scan_init_done) {
        AA.scan_pitch_deg = rad2degree(Gimbal.pitch.pitch_location_now);
        AA.scan_offset = 0.0f;
        AA.scan_yaw_dir = -1;
        AA.scan_init_done = 1;
    }

    float phys_now = rad2degree(Gimbal.pitch.pitch_location_now);

    /* Pitch 上下扫描 */
    if (Gimbal.pitch.turnover_pitch == 1) {
        AA.scan_pitch_deg += aa_scan_pitch_step_deg;
        if (phys_now >= (PITCH_MAX - 1.0f) || AA.scan_pitch_deg > (PITCH_MAX + 3.0f))
            Gimbal.pitch.turnover_pitch = 0;
    } else {
        AA.scan_pitch_deg -= aa_scan_pitch_step_deg;
        if (phys_now <= (PITCH_MIN + 1.0f) || AA.scan_pitch_deg < (PITCH_MIN - 3.0f))
            Gimbal.pitch.turnover_pitch = 1;
    }
    if (AA.scan_pitch_deg > PITCH_MAX + 3.0f) AA.scan_pitch_deg = PITCH_MAX + 3.0f;
    if (AA.scan_pitch_deg < PITCH_MIN - 3.0f) AA.scan_pitch_deg = PITCH_MIN - 3.0f;

    Global.Gimbal.input.pitch = AA.scan_pitch_deg;
    Gimbal.pitch.pitch_location_set = degree2rad(AA.scan_pitch_deg);

    /* 小Yaw扫描：相对大Yaw做位置往复 */
    AA.scan_offset += AA.scan_yaw_dir * aa_scan_yaw_step_deg;
    if (AA.scan_offset >= aa_scan_yaw_limit_deg)
    {
        AA.scan_offset = aa_scan_yaw_limit_deg;
        AA.scan_yaw_dir = -1;
    }
    if (AA.scan_offset <= -aa_scan_yaw_limit_deg)
    {
        AA.scan_offset = -aa_scan_yaw_limit_deg;
        AA.scan_yaw_dir = 1;
    }

    // 通过编码器获取小Yaw相对大Yaw的实际角度，反推大Yaw的世界系角度
    float big_yaw_world = Gimbal.small_yaw.small_yaw_location_now - small_yaw_relative_now;
    // 目标 = 大Yaw当前世界角 + 期望的相对偏移
    Gimbal.small_yaw.small_yaw_location_set = big_yaw_world + AA.scan_offset;

}

static void Scan_Reset_Internal(void)
{
    AA.scan_init_done = 0;
    AA.scan_pitch_deg = rad2degree(Gimbal.pitch.pitch_location_now);
    AA.scan_offset    = 0.0f;
    AA.scan_yaw_dir   = 0;
}

/**
 * @brief 上下沿边沿应用
 * 
 */
static void EdgeAction_Update(void)
{
    /* 上升沿 (0→1/2, 进入 track): 同步 setpoint */
    if (AA.control_req && !AA.control_req_last) 
    {
        AA.reacquire_freeze = 1;
        Gimbal.small_yaw.small_yaw_location_set = Gimbal.small_yaw.small_yaw_location_now;
        Global.Gimbal.input.yaw = Gimbal.small_yaw.small_yaw_location_now;
        Global.Auto.input.yaw_ff = 0;
        Global.Auto.input.pitch_ff = 0;
    }

    /* 下降沿 (1/2→0, 离开 track):准备进入扫描 */
    if (!AA.control_req && AA.control_req_last) 
    {
        Global.Auto.input.yaw_ff = 0;
        Global.Auto.input.pitch_ff = 0;
    }
}

static void Planner_Update(void)
{
    uint8_t auto_enabled = (Global.Auto.mode != NONE);

    if (!auto_enabled) 
    {
        AA.state = AIM_OFF;
        AA.is_scanning = 0;
    } 
    else if (AA.reacquire_freeze) 
    {
        AA.state = AIM_REACQUIRE;
            AA.is_scanning = 0;
        /* reacquire 拍: 不调 Auto_Control(), setpoint 保持当前位置 */
    } 
    else if (AA.control_req) 
    {
        AA.state = AIM_TRACK;
        AA.is_scanning = 0;
        Auto_Control();
    } 
    else 
    {
        AA.state = AIM_SCAN;
        if (!AA.is_scanning)
        Scan_Reset_Internal();
        Scan_Internal();
        AA.is_scanning = 1;
    }

    /* 清除冻结标志 */
    AA.reacquire_freeze = 0;
}

/**
 * @brief 自瞄路径
 * 
 */
static void FireGate_Update(void)
{
    Global.Shoot.mcu_fire_permit =
        (Global.Auto.mode != NONE)
        && AA.rx_fresh
        && (AA.mode_raw == CTRL_AIM_AND_FIRE)
        && (Global.Shoot.shoot_status == OK);
}

/**
 * @brief 填充 AA 内部调试量
 * 
 */
static void DebugFill_Update(void)
{
    AA.dbg_yaw_target_raw_deg = rad2degree(Global.Auto.input.shoot_yaw);
    if (AutoAim_IsScanning())
    {
        AA.dbg_yaw_cmd_deg = AA.scan_offset;
        AA.dbg_yaw_now_deg = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    }
    else
    {
        AA.dbg_yaw_cmd_deg = Global.Gimbal.input.yaw;
        AA.dbg_yaw_now_deg = imu_gimbal.yaw_cnt;
    }

    AA.dbg_yaw_ff_deg_s       = rad2degree(Global.Auto.input.yaw_ff);
    AA.dbg_encoder_relative   = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    AA.dbg_mcu_fire_permit    = (float)Global.Shoot.mcu_fire_permit;
}

/* =====================================================================
 *  公共接口 — 自瞄总入口 (freertos.c 每 1ms 调一次)
 * ===================================================================== */
void AutoAim_Update(void)
{
    /* 1. 链路收包 */
    taskENTER_CRITICAL();
    MINIPC_SyncShadowToWorkingCopy();
    taskEXIT_CRITICAL();

    /* 2. 语义解析 + 链路保底 + 数据锁存 */
    MINIPC_to_STM32();

    /* 3. 边沿动作 */
    EdgeAction_Update();

    /* 4. 状态决策 + 目标生成 / 扫描 */
    Planner_Update();

    /* 5. 发射许可 */
    FireGate_Update();

    /* 6. 调试量 */
    DebugFill_Update();

    /* 7. 保存本拍 control_req 用于下一拍边沿检测 */
    AA.control_req_last = AA.control_req;
}

/* =====================================================================
 *  查询接口 — 外围模块只用这几个，不自己拼条件
 * ===================================================================== */
uint8_t AutoAim_IsControllingGimbal(void)
{
    return (AA.state != AIM_OFF) ? 1 : 0;
}

uint8_t AutoAim_IsTracking(void)
{
    return (AA.state == AIM_TRACK) ? 1 : 0;
}

uint8_t AutoAim_IsScanning(void)
{
    return AA.is_scanning;
}

uint8_t AutoAim_ShouldBigYawFollow(void)
{
    return (AA.state != AIM_OFF) ? 1 : 0;
}
