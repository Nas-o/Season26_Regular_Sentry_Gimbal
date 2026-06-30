/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-18 05:42:42
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-05 07:40:27
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Shoot.hpp
 */
/**
 * @file  Shoot.hpp
 * @brief 发射机构控制器 —— C++类封装, 摩擦轮+拨弹盘
 */
#ifndef SHOOT_HPP_
#define SHOOT_HPP_

#ifdef __cplusplus
extern "C" 
{
#endif

    #include "CAN_receive_send.hpp"
    #include "pid.hpp"
    #include "motor.hpp"

    /* -------- 摩擦轮速度档位 -------- */
    enum ShootSpeed 
    {
        SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,
        SHOOT_SPEED_CLOSE = 0,
        SHOOT_SPEED_READY = FRIC_SPEED_REDAY,
        SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG,
    };

    /* -------- 拨弹速度档位 -------- */
    enum TriggerSpeed 
    {
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

    /* ---- C兼容接口 ---- */
    void Shoot_Init(void);
    void Shoot_Tasks(void);

#ifdef __cplusplus
}
#endif

/* ============================================================
 *  以下为 C++ 专用定义
 * ============================================================ */
#ifdef __cplusplus

namespace robot::shoot 
{

    class ShootController 
    {
        public:
            /* ---- 公有数据 (调试/外部可访问) ---- */
            pid_t fric_L_pid;
            pid_t fric_R_pid;
            pid_t trig_spd_pid;
            pid_t trig_pos_pid;

            float fric_spd_L = 0;
            float fric_spd_R = 0;
            float trig_spd = 0;
            float trig_current = 0;

            enum TriggerSpeed trig_spd_set;
            float fric_spd_set = 0;
            float current[3] = {};

            /* ---- 公有接口 ---- */
            void init();
            void task();

        private:
            void updateState();
            void selectTriggerMode();
            void calculate();
            void output();
    };

} // namespace robot::shoot

using robot::shoot::ShootController;
extern ShootController Shoot;

#endif /* __cplusplus */
#endif /* SHOOT_HPP_ */