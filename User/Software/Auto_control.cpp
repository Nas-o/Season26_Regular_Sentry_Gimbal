/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas-o 1319621819@qq.com
 * @LastEditTime: 2026-07-17 23:38:40
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Auto_control.cpp
 */

#include "Auto_control.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"

#include "IMU_updata.hpp"
#include "dm_imu.hpp"
#include "referee_system.hpp"
#include "UART_data_txrx.hpp"
#include "USB_VirCom.hpp"
#include "hipnuc_dec.hpp"
#include "pid.hpp"
#include "User_math.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "CRC8_CRC16.hpp"
#include "string.h"
#include <math.h>

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;
STM32ROS_data_t stm32send_1;
Sentry_cmd_t Sentry_cmd_1;
Referee_data_t Referee;

/* ========== 唯一自瞄全局状态 ========== */
AutoAim_t AutoAim;

uint32_t rx_timeout_ms    = 100;   // 链路新鲜度: 超时则不信任任何帧 
float yaw_reject_deg      = 60.0f; // 执行安全: 目标yaw与IMU差值超过此阈值 → 拒绝该帧
float yaw_ff_max_deg_s    = 100.0f; // 执行安全: yaw前馈速度限幅 (deg/s)
volatile uint32_t relative_timeout_ms = 8;
volatile uint32_t relative_tick = 0;
float big_yaw_ff_gain = 0.0f;
float big_yaw_ff_max_deg_s = 120.0f;

uint8_t RelativeAngle_IsFresh(void)
{
    if (relative_tick == 0)
    {
        return 0;
    }

    return (HAL_GetTick() - relative_tick <= relative_timeout_ms);
}
float aa_scan_yaw_limit_deg  = 45.0f; // 扫描yaw幅度 (deg)
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

// 将返回类型从 void 改为 uint8_t
uint8_t decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len)
{
    if (len != sizeof(Navigation_data_t) || buff[0] != 0xAA) {
        return 0; // 解析失败
    }

    uint16_t received_crc = (uint16_t)buff[len - 2] | ((uint16_t)buff[len - 1] << 8);
    uint16_t calculated_crc = Get_Modbus_CRC16(buff, len - 2);

    if (calculated_crc == received_crc) 
    {
        memcpy(target, buff, sizeof(Navigation_data_t));
        Navigation_online = 1;
        return 1; // 解析成功
    }
    
    return 0; // CRC 校验失败
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
    stm32send_1.contact_angle = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    stm32send_1.is_fire = Global.Shoot.trigger_mode;
    //stm32send_1.distance = fromMINIPC.distance;

     static uint8_t s_buff[sizeof(STM32ROS_data_t) + 3];
    s_buff[0] = 0xA5;
    memcpy(&s_buff[1], &stm32send_1, sizeof(STM32ROS_data_t));
    append_CRC16_check_sum(s_buff, sizeof(s_buff));

    /* Vircom_Send(s_buff, sizeof(s_buff)); */
    HAL_UART_Transmit_DMA(UART7_data.huart, s_buff, sizeof(s_buff));
    
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
    toMINIPC.mode =  MODE_AUTO_AIM;
    toMINIPC.yaw = degree2rad(imu_gimbal.yaw);//IMU_data.AHRS.yaw;
    toMINIPC.pitch = degree2rad(imu_gimbal.pitch);//IMU_data.AHRS.pitch;
    toMINIPC.yaw_vel = imu_gimbal.gyro[2];//IMU_data.gyro[2];
    toMINIPC.pitch_vel = imu_gimbal.gyro[1];//IMU_data.gyro[0];
    toMINIPC.q[0] = imu_gimbal.q[0];//IMU_data.AHRS.q[0];
    toMINIPC.q[1] = imu_gimbal.q[1];//IMU_data.AHRS.q[1];
    toMINIPC.q[2] = imu_gimbal.q[2];//IMU_data.AHRS.q[2];
    toMINIPC.q[3] = imu_gimbal.q[3];//IMU_data.AHRS.q[3];
    toMINIPC.bullet_speed = 24;//Referee.initial_speed;
    toMINIPC.enemy_color = 1;
    toMINIPC.bullet_count = Referee.launching_frequency;
    int len = (uint8_t *)&toMINIPC.crc16 - (uint8_t *)&toMINIPC;
    // 此时调用 CRC，无论结构体怎么变都永远正确
    toMINIPC.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.header, len, 0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

/**
 * @brief 角度差归一化到 [-180, 180]
 * 
 * @param x 
 * @return float 
 */
namespace {
float WrapDeltaDeg(float x)
{
    while (x >  180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}
} // namespace

void MINIPC_to_STM32(void)
{
    /* ---- 链路新鲜度 ---- */
    uint32_t age = HAL_GetTick() - minipc_last_rx_tick;
    AutoAim.rx_fresh = (minipc_last_rx_tick != 0 && age <= rx_timeout_ms) ? 1 : 0;

    /* ---- 检测新包 ---- */
    static uint32_t s_last_rx_seq = 0;
    uint8_t is_new_pkt = (minipc_rx_seq != s_last_rx_seq);
    s_last_rx_seq = minipc_rx_seq;

    /* ---- 语义决策直接取 mode ---- */
    AutoAim.mode_raw = fromMINIPC.mode;
    AutoAim.control_req = AutoAim.rx_fresh && (AutoAim.mode_raw != CTRL_NO_CONTROL);

    /* ---- 只在 fresh && control && 新包时更新目标数据 ---- */
    if (is_new_pkt && AutoAim.control_req)
    {
        Global.Auto.input.shoot_yaw    = fromMINIPC.yaw;
        Global.Auto.input.shoot_pitch  = fromMINIPC.pitch;
        Global.Auto.input.yaw_ff       = fromMINIPC.yaw_vel;
        Global.Auto.input.pitch_ff     = fromMINIPC.pitch_vel;
        Global.Auto.input.yaw_acc_ff   = fromMINIPC.yaw_acc;
        Global.Auto.input.pitch_acc_ff = fromMINIPC.pitch_acc;
    }


    /* ---- 非 TRACK 状态一律清前馈 (丢控制 / 扫描 / 重获目标均不用前馈) ---- */
    if (!AutoAim_IsTracking())
    {
        Global.Auto.input.yaw_ff       = 0.0f;
        Global.Auto.input.pitch_ff     = 0.0f;
        Global.Auto.input.yaw_acc_ff   = 0.0f;
        Global.Auto.input.pitch_acc_ff = 0.0f;
    }
}
#endif

namespace {
/**
 * @brief 跟踪
 * 
 */
void Auto_Control(void)
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

    AutoAim.target_yaw = target_yaw_cnt;
    AutoAim.target_pitch = -target_pitch_deg;
}


/**
 * @brief 扫描---入口切换
 *
 */
void Scan_Internal(void)
{
    const float PITCH_MAX = 20.0f;
    const float PITCH_MIN = PITCHI_MIN_ANGLE;
    float small_yaw_relative_now = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    /* 初始同步 */
    if (!AutoAim.scan_init_done) {
        AutoAim.scan_pitch_deg = rad2degree(Gimbal.pitch.pos_now);
        AutoAim.scan_offset = 0.0f;
        AutoAim.scan_yaw_dir = -1;
        AutoAim.scan_init_done = 1;
    }

    float phys_now = rad2degree(Gimbal.pitch.pos_now);

    /* Pitch 上下扫描 */
    if (Gimbal.pitch.turnover == 1) {
        AutoAim.scan_pitch_deg += aa_scan_pitch_step_deg;
        if (phys_now >= (PITCH_MAX - 1.0f) || AutoAim.scan_pitch_deg > (PITCH_MAX + 3.0f))
            Gimbal.pitch.turnover = 0;
    } else {
        AutoAim.scan_pitch_deg -= aa_scan_pitch_step_deg;
        if (phys_now <= (PITCH_MIN + 1.0f) || AutoAim.scan_pitch_deg < (PITCH_MIN - 3.0f))
            Gimbal.pitch.turnover = 1;
    }
    if (AutoAim.scan_pitch_deg > PITCH_MAX + 3.0f) AutoAim.scan_pitch_deg = PITCH_MAX + 3.0f;
    if (AutoAim.scan_pitch_deg < PITCH_MIN - 3.0f) AutoAim.scan_pitch_deg = PITCH_MIN - 3.0f;

    Global.Gimbal.input.pitch = AutoAim.scan_pitch_deg;
    Gimbal.pitch.pos_set = degree2rad(AutoAim.scan_pitch_deg);

    /* 小Yaw扫描：直接速度控制 */
    AutoAim.scan_offset = small_yaw_relative_now;
    float scan_yaw_speed_deg_s = 0.0f;
/*     if (toMINIPC.mode == MODE_AUTO_AIM)
    { */
        if (small_yaw_relative_now >= aa_scan_yaw_limit_deg)
            AutoAim.scan_yaw_dir = -1;
        else if (small_yaw_relative_now <= -aa_scan_yaw_limit_deg)
            AutoAim.scan_yaw_dir = 1;
        scan_yaw_speed_deg_s = aa_scan_yaw_speed_deg_s;
/*     }
    else
    {
        AutoAim.scan_yaw_dir = 0;
    } */
    

    Gimbal.yaw.spd_set = AutoAim.scan_yaw_dir * scan_yaw_speed_deg_s;

}

void Scan_Reset_Internal(void)
{
    AutoAim.scan_init_done = 0;
    AutoAim.scan_pitch_deg = rad2degree(Gimbal.pitch.pos_now);
    AutoAim.scan_offset    = 0.0f;
    AutoAim.scan_yaw_dir   = 0;
}

/**
 * @brief 上下沿边沿应用
 * 
 */
void EdgeAction_Update(void)
{
    /* 上升沿 (0→1/2, 进入 track): 同步 setpoint */
    if (AutoAim.control_req && !AutoAim.control_req_last) 
    {
        AutoAim.reacquire_freeze = 1;
        Gimbal.yaw.pos_set = Gimbal.yaw.pos_now;
        Global.Gimbal.input.yaw = Gimbal.yaw.pos_now;
        Global.Auto.input.yaw_ff = 0;
        Global.Auto.input.pitch_ff = 0;
    }

    /* 下降沿 (1/2→0, 离开 track):准备进入扫描 */
    if (!AutoAim.control_req && AutoAim.control_req_last) 
    {
        Global.Auto.input.yaw_ff = 0;
        Global.Auto.input.pitch_ff = 0;
    }
}

void Planner_Update(void)
{
    uint8_t auto_enabled = (Global.Auto.mode != NONE);

    if (!auto_enabled) 
    {
        AutoAim.state = AIM_OFF;
        AutoAim.is_scanning = 0;
    } 
    else if (AutoAim.reacquire_freeze) 
    {
        AutoAim.state = AIM_REACQUIRE;
            AutoAim.is_scanning = 0;
        /* reacquire 拍: 不调 Auto_Control(), setpoint 保持当前位置 */
    } 
    else if (AutoAim.control_req) 
    {
        AutoAim.state = AIM_TRACK;
        AutoAim.is_scanning = 0;
        Auto_Control();
    } 
    else 
    {
        AutoAim.state = AIM_SCAN;
        if (!AutoAim.is_scanning)
        Scan_Reset_Internal();
        Scan_Internal();
        AutoAim.is_scanning = 1;
    }

    /* 清除冻结标志 */
    AutoAim.reacquire_freeze = 0;
}

/**
 * @brief 自瞄路径
 * 
 */
void FireGate_Update(void)
{
    Global.Shoot.mcu_fire_permit =
        (Global.Auto.mode != NONE)
        && AutoAim.rx_fresh
        && (AutoAim.mode_raw == CTRL_AIM_AND_FIRE)
        && (Global.Shoot.shoot_status == OK);
}

/**
 * @brief 填充 AA 内部调试量
 * 
 */
void DebugFill_Update(void)
{
    AutoAim.dbg_yaw_target_raw_deg = rad2degree(Global.Auto.input.shoot_yaw);
    if (AutoAim_IsScanning())
    {
        AutoAim.dbg_yaw_cmd_deg = AutoAim.scan_offset;
        AutoAim.dbg_yaw_now_deg = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    }
    else
    {
        AutoAim.dbg_yaw_cmd_deg = Global.Gimbal.input.yaw;
        AutoAim.dbg_yaw_now_deg = imu_gimbal.yaw_cnt;
    }

    AutoAim.dbg_yaw_ff_deg_s       = rad2degree(Global.Auto.input.yaw_ff);
    AutoAim.dbg_encoder_relative   = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    AutoAim.dbg_mcu_fire_permit    = (float)Global.Shoot.mcu_fire_permit;
}
} // namespace

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
    AutoAim.control_req_last = AutoAim.control_req;

    /* 8.丢包检测 */
    float chk[4] = {imu_gimbal.gyro[0],imu_gimbal.gyro[2],fromMINIPC.yaw_vel, fromMINIPC.pitch_vel};
    LostPkg_Detect(chk, 4);
}

/* =====================================================================
 *  查询接口 — 外围模块只用这几个，不自己拼条件
 * ===================================================================== */
uint8_t AutoAim_IsControllingGimbal(void)
{
    return (AutoAim.state != AIM_OFF) ? 1 : 0;
}

uint8_t AutoAim_IsTracking(void)
{
    return (AutoAim.state == AIM_TRACK) ? 1 : 0;
}

uint8_t AutoAim_IsScanning(void)
{
    return AutoAim.is_scanning;
}

uint8_t AutoAim_ShouldBigYawFollow(void)
{
    return (AutoAim.state != AIM_OFF) ? 1 : 0;
}

/* =====================================================================
 *  通信丢包实时检测
 *  用法: 每次收到新数据时调用, 传入当前值; 函数内部与上次值比较,
 *        若值未更新(相同)则认为丢包, 对应计数器自增.
 *        最多同时监测 4 个变量 (通道 0~3).
 * ===================================================================== */
uint32_t lost_pkg_cnt[4] = {0};          // 4个通道的丢包计数

namespace {
float lost_pkg_last_val[4] = {0};        // 上一次各通道的值
uint8_t lost_pkg_init[4] = {0};          // 初始化标志
} // namespace

/**
 * @brief 通信丢包检测 — 传入最多4个变量的当前值
 * @param vals   指向浮点数组, 包含各通道当前值
 * @param n      本次传入的通道数 (1~4)
 * @note  若某通道当前值与上次完全相同, 则认为该通道丢包, lost_pkg_cnt[ch]++
 *        首次调用时仅记录初值, 不计丢包.
 */
void LostPkg_Detect(const float vals[], uint8_t n)
{
    if (n > 4) n = 4;
    for (uint8_t i = 0; i < n; i++)
    {
        if (!lost_pkg_init[i])
        {
            lost_pkg_last_val[i] = vals[i];
            lost_pkg_init[i] = 1;
            continue;
        }
        if (vals[i] == lost_pkg_last_val[i])
        {
            lost_pkg_cnt[i]++;
        }
        lost_pkg_last_val[i] = vals[i];
    }
}

/**
 * @brief 重置指定通道的丢包计数
 * @param ch 通道号 0~3, 传入 0xFF 则全部重置
 */
void LostPkg_Reset(uint8_t ch)
{
    if (ch == 0xFF)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            lost_pkg_cnt[i] = 0;
            lost_pkg_init[i] = 0;
        }
    }
    else if (ch < 4)
    {
        
        lost_pkg_cnt[ch] = 0;
        lost_pkg_init[ch] = 0;
    }
}
