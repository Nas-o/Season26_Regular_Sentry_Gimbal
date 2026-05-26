/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-12-25 19:38:32
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-19 23:04:36
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Gimbal.h
 */
/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       gimbal.c/h
  * @brief      云台控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建云台代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#ifndef __GIMBAL_DIRECT_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __GIMBAL_DIRECT_H__

#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "ramp_generator.hpp"
#include "lqr_controller.hpp"

#include "motor.hpp"

/* 小yaw控制模式选择: 0=PID(回退), 1=LQR/LQI */
#define SMALL_YAW_USE_LQR  0

/* Pitch LQR外环: 0=保持现有MIT-PD, 1=LQR外环+MIT内层 (小yaw验证稳定后再开) */
#define PITCH_USE_LQR      0


/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT MAX_CURRENT


/*内部数据类型*/
typedef struct
{

  struct 
  {
    float pitch_speed_now;
    float pitch_location_now;
    float pitch_speed_set;
    float pitch_location_set;
    float current;
    int turnover_pitch;
    // MIT模式位置控制参数
    float kp;  // 位置比例增益
    float kd;  // 速度阻尼增益
    float k_gravity; // 重力前馈系数
    float pitch_offset;
    uint8_t init_flag; // 初始化完成标志
    float gravity_compensation; //重力补偿
    float gravity_integral;       // 外环积分力矩补偿（消除静摩擦/模型残差导致的稳态偏差）
    float ki_gravity;             // 积分增益
    float gravity_integral_limit; // 积分限幅 (N·m)
    LQR_Controller_t lqr;         // Pitch LQR 外环 (输出力矩指令, 叠加重力补偿后送 MIT)
  }pitch;

  struct
  {
    pid_t small_yaw_speed_pid;
    pid_t small_yaw_location_pid;
    pid_t small_yaw_auto_speed_pid;
    pid_t small_yaw_auto_location_pid;
    LQR_Controller_t lqr;              // LQR/LQI 状态反馈控制器
    LQR_Controller_t lqr_auto;         // 自瞄模式 LQR (可独立调参)
    float small_yaw_speed_now;
    float small_yaw_speed_raw;          // 滤波前的原始速度
    float small_yaw_speed_lpf_alpha;    // 一阶低通滤波系数 (0~1)
    float small_yaw_location_now;
    float small_yaw_speed_set;
    float small_yaw_location_set;
    float current;
    int turnover_yaw;
    float small_from_big_feedforward;
  }small_yaw;

  struct 
  {
    pid_t big_yaw_speed_pid;
    pid_t big_yaw_location_pid;
    pid_t big_yaw_auto_speed_pid;
    pid_t big_yaw_auto_location_pid;
    float big_yaw_speed_now;
    float big_yaw_location_now;
    float big_yaw_speed_set;
    float big_yaw_location_set;
    float current;
    float tor;
  }big_yaw;  
    // 斜坡
    RampGenerator pitch_ramp;
    RampGenerator yaw_ramp;

}Gimbal_t;

void Gimbal_Init();
void Gimbal_Tasks();
void Gimbal_SetPitchAngle(float angle);
void Gimbal_SetYawAngle(float angle);
void Gimbal_Limit(float pitch_up_angle, float pitch_down_angle, float yaw_L_angle, float yaw_R_angle);
extern Gimbal_t Gimbal;
// 添加 volatile 防止编译器优化导致的变量读取不同步
extern volatile float relative_angle;

#ifdef __cplusplus
}
#endif
#endif 

