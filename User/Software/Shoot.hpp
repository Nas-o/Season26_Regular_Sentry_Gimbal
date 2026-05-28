/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-18 05:42:42
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-27 19:17:14
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Shoot.hpp
 */
/**
 * @file  Shoot.hpp
 * @brief 发射机构控制器 —— 摩擦轮+拨弹盘
 */
#ifndef SHOOT_HPP_
#define SHOOT_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "motor.hpp"

/* -------- 摩擦轮速度档位 -------- */
enum shoot_speed_e {
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,
    SHOOT_SPEED_CLOSE = 0,
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG,
};

/* -------- 拨弹速度档位 -------- */
enum trigger_speed_e {
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,
};

/* -------- 电机操作宏 -------- */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)

#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

/* ============================================================
 *  发射机构数据
 * ============================================================ */
typedef struct {
    // PID
    pid_t fric_L_pid;
    pid_t fric_R_pid;
    pid_t trig_spd_pid;
    pid_t trig_pos_pid;

    // 状态
    float fric_spd_L;       // 左摩擦轮当前转速
    float fric_spd_R;       // 右摩擦轮当前转速
    float trig_spd;         // 拨弹当前转速
    float trig_current;     // 拨弹当前电流

    // 目标
    enum trigger_speed_e trig_spd_set;
    float fric_spd_set;     // 摩擦轮目标转速 (含自适应偏移)
    float current[3];       // 输出电流
} Shoot_t;

/* -------- 外部接口 -------- */
void Shoot_Init();
void Shoot_Tasks();

#ifdef __cplusplus
}
#endif
#endif // SHOOT_HPP_