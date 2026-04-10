/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-04-08 05:13:24
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Chassis.h
 */

/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       chassis.c/h
  * @brief      底盘控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建底盘代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "CAN_receive_send.h"

#include "pid.h"
#include "User_math.h"
#include "ramp_generator.h"

#include "motor.h"

#define FOLLOW_LPF_ALPHA 0.15f   // 越小滤波越强，0.1~0.3 之间调
/*电机配置*/

#define CHASSISMotor_init(type, id)  DJIMotor_Init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)

/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

/* 舵向电机零点初始化 */
#define TURN_FR_ECD 7560
#define TURN_FL_ECD 4097
#define TURN_BL_ECD 5435
#define TURN_BR_ECD 6181

// 轮子向前时6020编码器的值
#define X_AXIS_ECD_FL 4096                        
#define X_AXIS_ECD_FR 7600
#define X_AXIS_ECD_BL 2766
#define X_AXIS_ECD_BR 6190

/* 功率限制模型参数 (需根据实际底盘辨识调整) */
// P = k0*I*w + k1*|w| + k2*(k0*I)^2 + k3

// 轮向电机 (M3508 + 19:1)
#define WHEEL_K0 0.214f
#define WHEEL_K1 0.132f
#define WHEEL_K2 3.47f
#define WHEEL_K3 7.8f   

// 舵向电机 (GM6020)
#define STEER_K0 0.741f
#define STEER_K1 0.005f
#define STEER_K2 12.98f
#define STEER_K3 7.8f

/* 功率限制器结构体 */
typedef struct
{
    // 模型参数
    float k0, k1, k2, k3;
    
    // 状态量
    float limit_power_w;     // 当前生效的功率上限 (裁判限制 + 电容补偿)
    float predict_power; // 预测的总功率
    
    // 调试/监控用
    float scaling_factor;    // 电流缩放系数 (0.0 ~ 1.0)
    
} PowerLimiter_t;

/*内部数据类型*/
typedef struct
{
    /**************************以下是轮向电机结构体定义***************************/

    struct
    {
        pid_t chassis_speed_pid_forward_FL;
        float V_FL;
        float opposite_direction_FL;
        float target_velocity_FL, current_velocity_FL;
        int16_t wheel_current_FL;
    } forward_FL;
    struct
    {
        pid_t chassis_speed_pid_forward_FR;       
        float V_FR;
        float opposite_direction_FR;
        float target_velocity_FR, current_velocity_FR;
        int16_t wheel_current_FR;
    } forward_FR;
    struct
    {
        pid_t chassis_speed_pid_forward_BL;
        float V_BL;
        float opposite_direction_BL;
        float target_velocity_BL, current_velocity_BL;
        int16_t wheel_current_BL;
    } forward_BL;
    struct
    {
        pid_t chassis_speed_pid_forward_BR;
        float V_BR;
        float opposite_direction_BR;
        float target_velocity_BR, current_velocity_BR;
        int16_t wheel_current_BR;
    } forward_BR;

    /**************************以下是舵向电机结构体定义***************************/


    struct
    {
        pid_t chassis_location_pid_turn_FL;
        pid_t chassis_speed_pid_turn_FL;
        float set, now;
        float set_turn_FL_speed;
        float target_angle_turn_FL;
        float set_angle_FL;
        float set_ECD_FL;
        int16_t current_steer_FL;
    } turn_FL;
    struct
    {
        pid_t chassis_location_pid_turn_FR;
        pid_t chassis_speed_pid_turn_FR;
        float set, now;
        float set_turn_FR_speed;
        float target_angle_turn_FR;
        float set_angle_FR;
        float set_ECD_FR;
        int16_t current_steer_FR;
    } turn_FR;
    struct
    {
        pid_t chassis_location_pid_turn_BL;
        pid_t chassis_speed_pid_turn_BL;
        float set, now;
        float set_turn_BL_speed;
        float target_angle_turn_BL;
        float set_angle_BL;
        float set_ECD_BL;
        int16_t current_steer_BL;
    } turn_BL;
    struct
    {
        pid_t chassis_location_pid_turn_BR;
        pid_t chassis_speed_pid_turn_BR;
        float set, now;
        float set_turn_BR_speed;
        float target_angle_turn_BR;
        float set_angle_BR;
        float set_ECD_BR;
        int16_t current_steer_BR;
    } turn_BR;
    struct
    {
        float x, y, r, yaw;
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s
    } speed;

    struct
    {
        float x, y, r, big_yaw;
    } speed_RC;

    struct
    {
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s^2
    } acc;

    struct
    {
        int refresh_interval;
        int smaller_than_2_count;
        float valve;
    }random;

    pid_t chassis_follow_pid;
    uint8_t is_open_cap;
    float relative_angle;

    PowerLimiter_t limiter_wheel; // 轮向限制器
    PowerLimiter_t limiter_steer; // 舵向限制器
    
}Chassis_t;
extern Chassis_t Chassis;

/*外部调用*/
void Chassis_Init();
void Chassis_Tasks();
void Chassis_CAN_SendAll(void);
void Chassis_SetX(float x);
void Chassis_SetY(float y);
void Chassis_SetR(float r);
void Chassis_SetAccel(float acc);
void Mode_Switch();
float Random_Spin(float min, float max);
float Generate_Random_Float(float min, float max);
float Variable_Spin_Update(void);
void Chassis_PowerLimit_Update(float max_power);
#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif
