/**
 * @file  Chassis.hpp
 * @brief 底盘控制器 —— 云台板侧 (指令生成, 通过CAN发送到底盘板)
 */
#ifndef CHASSIS_HPP_
#define CHASSIS_HPP_

#ifdef __cplusplus
extern "C" 
{
#endif

    #include "CAN_receive_send.hpp"
    #include "pid.hpp"
    #include "User_math.hpp"
    #include "ramp_generator.hpp"
    #include "motor.hpp"

    /* -------- 电机宏 -------- */
    #define CHASSISMotor_init(type, id)  DJIMotor_Init(type, id)
    #define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
    #define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)

    #define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
    #define CHASSISMOTOR_T_A DJIMOTOR_T_A


    /* ---- C兼容接口 ---- */
    void  Chassis_Init(void);
    void  Chassis_Tasks(void);
    void  Chassis_SetX(float x);
    void  Chassis_SetY(float y);
    void  Chassis_SetR(float r);
    void  Chassis_SetAccel(float acc);
    void  Mode_Switch(void);
    float Random_Spin(float min, float max);
    float Generate_Random_Float(float min, float max);
    float Variable_Spin_Update(void);

#ifdef __cplusplus
}
#endif

/* ============================================================
 *  以下为 C++ 专用定义
 * ============================================================ */
#ifdef __cplusplus

namespace robot::chassis 
{

    /* ---- 底盘参数常量 ---- */
    constexpr float kFollowLpfAlpha = 0.15f;   // 底盘跟随低通滤波系数
    constexpr int   kChassisTaskTime = 1;      // 底盘任务周期 (ms)

    /** @brief 底盘运动输出  */
    struct ChassisLinkMotion 
    {
        float x = 0;   // X轴线速度 (rad/s, 世界坐标系)
        float y = 0;   // Y轴线速度 (rad/s, 世界坐标系)
        float r = 0;   // 旋转角速度 (rad/s)
    };

    /**
     * @brief 底盘控制器类 —— 云台板侧指令生成
     * @note  仅负责运动指令的计算与模式管理,
     *        实际电机驱动在底盘板上完成 (通过CAN通信)
     */
    class ChassisController 
    {
        public:
            struct 
            {
                float x = 0, y = 0, r = 0, yaw = 0;        // 当前期望速度
                float now_x = 0, now_y = 0, now_r = 0;     // 实际反馈速度
                float max_x = 0, max_y = 0, max_r = 0;     // 最大允许速度
            } speed;

            struct
            {
                float x = 0, y = 0, r = 0, big_yaw = 0;    // 遥控器直接给定的速度
            } speed_RC;

            struct 
            {
                float now_x = 0, now_y = 0, now_r = 0;     // 当前加速度
                float max_x = 0, max_y = 0, max_r = 0;     // 最大加速度限制
            } acc;

            struct 
            {
                int   interval = 250;   // 随机换向间隔 (调用次数)
                int   low_count = 0;    // 连续低速计数
                float value = 0;        // 当前随机转速值
            } random;

            pid_t  follow_pid;          // 底盘跟随云台PID
            uint8_t is_open_cap = 0;   // 超级电容开启标志
            float  relative_angle = 0; // 小yaw相对底盘角度 (deg)

            void init();                                    // 初始化
            void task();                                    // 1kHz控制主循环
            void setX(float x);                            // 设置X轴目标速度
            void setY(float y);                            // 设置Y轴目标速度
            void setR(float r);                            // 设置旋转速度
            void setAccel(float acc);                      // 设置加速度 (预留)
            void modeSwitch();                             // 模式切换与速度解算

            float randomSpin(float min, float max);        // 随机小陀螺速度
            float variableSpinUpdate();                    // 变速小陀螺更新

            static float angleLimit(float angle);          // 角度归一化到[-180,180]
            static float obtainModulusNormalization(float x, float modulus); // 模值归一化
            static float generateRandomFloat(float min, float max);          // 随机浮点数
    };

} 

using robot::chassis::ChassisController;
extern ChassisController Chassis;
extern robot::chassis::ChassisLinkMotion ChassisLinkMotion;

typedef robot::chassis::ChassisLinkMotion ChassisLinkMotion_t;

#endif /* __cplusplus */
#endif /* CHASSIS_HPP_ */
