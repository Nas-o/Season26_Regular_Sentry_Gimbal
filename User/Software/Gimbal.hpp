/**
 * @file  Gimbal.hpp
 * @brief 云台控制器 —— C++类封装
 */
#ifndef GIMBAL_HPP_
#define GIMBAL_HPP_

#ifdef __cplusplus
extern "C" 
{
#endif

    #include "CAN_receive_send.hpp"
    #include "pid.hpp"
    #include "ramp_generator.hpp"
    #include "lqr_controller.hpp"
    #include "motor.hpp"

    /* -------- 控制策略开关 -------- */
    #define SMALL_YAW_USE_LQR  0   // 小yaw: 0=PID, 1=LQR/LQI
    #define PITCH_USE_LQR      0   // Pitch外环: 0=MIT-PD, 1=LQR+MIT

    /* -------- 电机参数 -------- */
    #define GIMBAL_MAX_CURRENT MAX_CURRENT

    void Gimbal_Init(void);
    void Gimbal_Tasks(void);
    void Gimbal_SetPitchAngle(float angle);
    void Gimbal_SetYawAngle(float angle);
    void Gimbal_Limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace robot::gimbal 
{

/* ---- 轴控制数据结构 ---- */

/** @brief Pitch轴数据 */
struct PitchAxis 
{
    float spd_now = 0;      // 当前角速度 (rad/s)
    float pos_now = 0;      // 当前角度 (rad)
    float spd_set = 0;      // 目标角速度
    float pos_set = 0;      // 目标角度
    float current = 0;      // 输出电流/力矩
    int   turnover = 0;     // 圈数计数

    // MIT 参数
    float kp = 0;           // 位置比例增益
    float kd = 0;           // 速度阻尼增益
    float k_gravity = 0;    // 重力前馈系数
    float offset = 0;       // IMU-电机坐标差
    uint8_t inited = 0;     // 初始化完成标志

    // 重力补偿
    float grav_comp = 0;    // cos模型补偿
    float grav_integral = 0;// 积分力矩修正
    float ki_grav = 0;      // 积分增益
    float grav_int_limit = 0;// 积分限幅 (N·m)

    LQR_Controller_t lqr;   // LQR外环 (PITCH_USE_LQR=1时生效)
};

/** @brief Yaw轴数据 */
struct YawAxis 
{
    // PID 串级
    pid_t spd_pid;           // 速度环
    pid_t pos_pid;           // 位置环
    pid_t auto_spd_pid;      // 自瞄速度环
    pid_t auto_pos_pid;      // 自瞄位置环

    // LQR
    LQR_Controller_t lqr;        // 手动/导航 LQR
    LQR_Controller_t lqr_auto;   // 自瞄 LQR

    // 状态
    float spd_now = 0;       // 滤波后角速度 (deg/s)
    float spd_raw = 0;       // 原始角速度
    float lpf_alpha = 1.0f;  // 低通滤波系数 (0~1)
    float pos_now = 0;       // 当前角度 (deg)
    float spd_set = 0;       // 目标角速度
    float pos_set = 0;       // 目标角度
    float current = 0;       // 输出电流
    int   turnover = 0;      // 圈数计数
    float big_ff = 0;        // 大yaw前馈补偿
};

/** @brief 大Yaw轴数据  */
struct BigYawAxis 
{
    pid_t spd_pid;
    pid_t pos_pid;
    pid_t auto_spd_pid;
    pid_t auto_pos_pid;
    float spd_now = 0;
    float pos_now = 0;
    float spd_set = 0;
    float pos_set = 0;
    float current = 0;
    float torque = 0;
};

/* ============================================================
 *  GimbalController 类 —— 云台控制器
 * ============================================================ */
class GimbalController 
{
    public:
        /* ---- 公有数据 ---- */
        PitchAxis   pitch;
        YawAxis     yaw;       // 小yaw
        BigYawAxis  big_yaw;

        /* ---- 公有接口 ---- */
        void init();
        void task();
        void setPitchAngle(float angle);
        void setYawAngle(float angle);
        void limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r);

    private:
        /* ---- 控制周期 ---- */
        static constexpr float kCtrlDt = 0.001f;  // 1kHz

        /* ---- 模式切换状态 ---- */
        uint8_t last_mode_ = 0;
        uint8_t last_chassis_mode_ = 0;
        uint8_t first_run_ = 1;

        /* ---- 私有方法 ---- */
        void update();
        void calculate();
        void control();
        float getBigYawFF_DegS();

        template<typename T>
        static constexpr T clamp(T val, T low, T high) 
        {
            return (val < low) ? low : (val > high) ? high : val;
        }
};

} // namespace robot::gimbal

/* ---- 全局实例 ---- */
using robot::gimbal::GimbalController;
using robot::gimbal::PitchAxis;
using robot::gimbal::YawAxis;
using robot::gimbal::BigYawAxis;
extern GimbalController Gimbal;
extern volatile float relative_angle;

#endif /* __cplusplus */
#endif /* GIMBAL_HPP_ */

