/**
 * @file  Gimbal.hpp
 * @brief 云台控制器 —— 包含Pitch/Yaw轴数据与控制接口
 */
#ifndef GIMBAL_HPP_
#define GIMBAL_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "ramp_generator.hpp"
#include "lqr_controller.hpp"
#include "motor.hpp"

/* -------- 控制策略开关 -------- */
#define SMALL_YAW_USE_LQR  0   // 小yaw: 0=PID(回退), 1=LQR/LQI
#define PITCH_USE_LQR      0   // Pitch外环: 0=MIT-PD, 1=LQR+MIT

/* -------- 电机参数 -------- */
#define GIMBAL_MAX_CURRENT MAX_CURRENT

/* ============================================================
 *  轴控制数据结构 (简化命名: 结构体已表达所属轴, 字段无需重复)
 * ============================================================ */

/** @brief Pitch轴数据 (DM4310 MIT模式) */
typedef struct {
    float spd_now;          // 当前角速度 (rad/s)
    float pos_now;          // 当前角度 (rad)
    float spd_set;          // 目标角速度
    float pos_set;          // 目标角度
    float current;          // 输出电流/力矩
    int   turnover;         // 圈数计数

    // MIT 参数
    float kp;               // 位置比例增益
    float kd;               // 速度阻尼增益
    float k_gravity;        // 重力前馈系数
    float offset;           // IMU-电机坐标差
    uint8_t inited;         // 初始化完成标志

    // 重力补偿
    float grav_comp;        // cos模型补偿
    float grav_integral;    // 积分力矩修正
    float ki_grav;          // 积分增益
    float grav_int_limit;   // 积分限幅 (N·m)

    LQR_Controller_t lqr;   // LQR外环 (PITCH_USE_LQR=1时生效)
} PitchAxis;

/** @brief Yaw轴数据 (小yaw GM6020) */
typedef struct {
    // PID 串级
    pid_t spd_pid;           // 速度环
    pid_t pos_pid;           // 位置环
    pid_t auto_spd_pid;      // 自瞄速度环
    pid_t auto_pos_pid;      // 自瞄位置环

    // LQR
    LQR_Controller_t lqr;        // 手动/导航 LQR
    LQR_Controller_t lqr_auto;   // 自瞄 LQR

    // 状态
    float spd_now;           // 滤波后角速度 (deg/s)
    float spd_raw;           // 原始角速度
    float lpf_alpha;         // 低通滤波系数 (0~1)
    float pos_now;           // 当前角度 (deg)
    float spd_set;           // 目标角速度
    float pos_set;           // 目标角度
    float current;           // 输出电流
    int   turnover;          // 圈数计数
    float big_ff;            // 大yaw前馈补偿
} YawAxis;

/** @brief 大Yaw轴数据 (通过CAN发送到底盘) */
typedef struct {
    pid_t spd_pid;
    pid_t pos_pid;
    pid_t auto_spd_pid;
    pid_t auto_pos_pid;
    float spd_now;
    float pos_now;
    float spd_set;
    float pos_set;
    float current;
    float torque;
} BigYawAxis;

/** @brief 云台总数据 */
typedef struct {
    PitchAxis   pitch;
    YawAxis     yaw;       // 小yaw
    BigYawAxis  big_yaw;

    RampGenerator pitch_ramp;
    RampGenerator yaw_ramp;
} Gimbal_t;

/* -------- 外部接口 -------- */
void Gimbal_Init();
void Gimbal_Tasks();
void Gimbal_SetPitchAngle(float angle);
void Gimbal_SetYawAngle(float angle);
void Gimbal_Limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r);

extern Gimbal_t Gimbal;
extern volatile float relative_angle;

#ifdef __cplusplus
}
#endif
#endif // GIMBAL_HPP_

