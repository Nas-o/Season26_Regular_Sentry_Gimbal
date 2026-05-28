/**
 * @file  Chassis.hpp
 * @brief 底盘控制器 —— 云台板侧 (指令生成, 通过CAN发送到底盘板)
 */
#ifndef CHASSIS_HPP_
#define CHASSIS_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "User_math.hpp"
#include "ramp_generator.hpp"
#include "motor.hpp"

/* -------- 底盘参数 -------- */
#define FOLLOW_LPF_ALPHA  0.15f
#define CHASSIS_TASK_TIME 1     // ms

/* -------- 电机宏 -------- */
#define CHASSISMotor_init(type, id)  DJIMotor_Init(type, id)
#define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)

#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

/* -------- 舵向零点 -------- */
#define TURN_FR_ECD 7560
#define TURN_FL_ECD 4097
#define TURN_BL_ECD 5435
#define TURN_BR_ECD 6181

#define X_AXIS_ECD_FL 4096
#define X_AXIS_ECD_FR 7600
#define X_AXIS_ECD_BL 2766
#define X_AXIS_ECD_BR 6190

/* -------- 功率限制模型参数 -------- */
#define WHEEL_K0 0.214f
#define WHEEL_K1 0.132f
#define WHEEL_K2 3.47f
#define WHEEL_K3 7.8f

#define STEER_K0 0.741f
#define STEER_K1 0.005f
#define STEER_K2 12.98f
#define STEER_K3 7.8f

/* ============================================================
 *  数据结构
 * ============================================================ */

typedef struct {
    float k0, k1, k2, k3;
    float limit_power_w;
    float predict_power;
    float scaling_factor;
} PowerLimiter_t;

typedef struct {
    float x;
    float y;
    float r;
} ChassisLinkMotion_t;

/** @brief 底盘数据 (云台板侧: 仅保留指令生成所需字段) */
typedef struct {
    struct {
        float x, y, r, yaw;
        float now_x, now_y, now_r;
        float max_x, max_y, max_r;
    } speed;

    struct {
        float x, y, r, big_yaw;
    } speed_RC;

    struct {
        float now_x, now_y, now_r;
        float max_x, max_y, max_r;
    } acc;

    struct {
        int   interval;       // 随机刷新间隔
        int   low_count;      // 低速计数
        float value;          // 当前随机值
    } random;

    pid_t  follow_pid;
    uint8_t is_open_cap;
    float  relative_angle;

    PowerLimiter_t limiter_wheel;
    PowerLimiter_t limiter_steer;
} Chassis_t;

/* -------- 外部接口 -------- */
extern Chassis_t Chassis;
extern ChassisLinkMotion_t ChassisLinkMotion;

void  Chassis_Init();
void  Chassis_Tasks();
void  Chassis_SetX(float x);
void  Chassis_SetY(float y);
void  Chassis_SetR(float r);
void  Chassis_SetAccel(float acc);
void  Mode_Switch();
float Random_Spin(float min, float max);
float Generate_Random_Float(float min, float max);
float Variable_Spin_Update(void);
void  Chassis_PowerLimit_Update(float max_power);

#ifdef __cplusplus
}
#endif
#endif // CHASSIS_HPP_
