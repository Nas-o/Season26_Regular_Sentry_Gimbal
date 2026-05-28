/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-27
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Gimbal.cpp
 * @brief  云台控制器 —— 初始化、数据更新、控制解算、电机输出
 * @note   包括初始化，数据更新、控制量计算与直接控制量设置
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建云台代码结构
 *  V2.0.0   2026.05.27       Refactor         2.C++重构: 匿名命名空间, constexpr, 命名简化
 */
#include "Gimbal.hpp"
#include "Global_status.hpp"

#include "User_math.hpp"
#include "remote_control.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"
#include "hipnuc_dec.hpp"
#include "motor.hpp"
#include "vofa+.hpp"
#include "Auto_control.hpp"
#include "lqr_controller.hpp"

Gimbal_t Gimbal;
volatile float relative_angle;

/* ============== 公共接口 ============== */

void Gimbal_Limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r)
{
    if (Gimbal.pitch.pos_set <= pitch_up && Gimbal.pitch.pos_set < Gimbal.pitch.pos_now)
        Gimbal.pitch.pos_set = pitch_up;
    else if (Gimbal.pitch.pos_set >= pitch_down && Gimbal.pitch.pos_set > Gimbal.pitch.pos_now)
        Gimbal.pitch.pos_set = pitch_down;
}

/* ============== 内部实现 (匿名命名空间, 仅本文件可见) ============== */
namespace {

constexpr float kCtrlDt = 0.001f;  // 控制周期 (s), 1kHz

template<typename T>
constexpr T Clamp(T val, T lo, T hi) {
    return (val < lo) ? lo : (val > hi) ? hi : val;
}

/**
 * @brief 大yaw底盘前馈 → 小yaw速度参考 (deg/s)
 * @note  对板数据不新鲜时返回0, 执行安全限幅
 */
float GetBigYawFF_DegS()
{
    if (!RelativeAngle_IsFresh())
        return 0.0f;

    // 下板发来的是 deg/s，这里不要再做 rad2degree
    float ff = -Gimbal.yaw.big_ff * aa_big_yaw_ff_gain;
    if (fabsf(ff) > aa_big_yaw_ff_max_deg_s)
        return 0.0f;

    return ff;
}

} // namespace (utilities)

/*-------------------- Init --------------------*/

/**
 * @brief  云台初始化
 * @note   电机初始化 → PID/LQR参数 → 零点设定 → 校准变量
 */
void Gimbal_Init()
{
    // 云台电机初始化
    DMMotor_Init(DM_4310, PITCHMotor);
    DJIMotor_Init(DJI_GM6020, SMALLYAWMotor);
    
    /*------MIT模式参数初始化------*/
    Gimbal.pitch.kp = 18.0f;              // 位置比例增益
    Gimbal.pitch.kd = 1.2f;               // 速度阻尼增益
    Gimbal.pitch.k_gravity = 0.891800784f; // 重力前馈系数

    /*------Pitch LQR 外环 (PITCH_USE_LQR=1 时生效)------*/
    // 暂用 2 态 LQR (不含积分, 因为已有 gravity_integral 做稳态修正)
    // 状态 = [pitch_angle_rad, pitch_rate_rad_s], 输出 = 力矩(N·m)
    {
        float K_pitch[2] = { 18.0f, 1.2f };
        LQR_Init(&Gimbal.pitch.lqr, K_pitch, 2, kCtrlDt, 0.0f, 3.0f);
    }
    
    /*------小Yaw PID 串级初始化------*/
    PID_Set(&Gimbal.yaw.spd_pid,      65.0f, 0.01f, 3.0f, 0.0f, GIMBAL_MAX_CURRENT, GIMBAL_MAX_CURRENT);
    PID_Set(&Gimbal.yaw.auto_spd_pid, 85.0f, 0.01f, 3.0f, 0.0f, GIMBAL_MAX_CURRENT, GIMBAL_MAX_CURRENT);
    PID_Set(&Gimbal.yaw.pos_pid,      18.0f, 0.0f,  1.0f, 0.0f, GIMBAL_MAX_CURRENT, 1000);
    PID_Set(&Gimbal.yaw.auto_pos_pid, 18.0f, 0.0f,  1.0f, 0.0f, GIMBAL_MAX_CURRENT, 1000);

    // 小yaw速度一阶低通滤波系数 (0~1, 越小滤波越强)
    Gimbal.yaw.lpf_alpha = 1.0f;
    Gimbal.yaw.spd_now = 0.0f;
    Gimbal.yaw.spd_raw = 0.0f;

    /*------小Yaw LQR/LQI 初始化------*/
    // K 增益由 MATLAB dlqr/lqr 离线求得, 状态 = [angle_deg, rate_deg_s, integral]
    {
        float K_manual[3] = { 120.0f, 8.0f, 60.0f };
        LQR_Init(&Gimbal.yaw.lqr, K_manual, 3, kCtrlDt, 50.0f, GIMBAL_MAX_CURRENT);

        // 自瞄模式 (可独立调参, 通常更激进)
        float K_auto[3] = { 150.0f, 10.0f, 80.0f };
        LQR_Init(&Gimbal.yaw.lqr_auto, K_auto, 3, kCtrlDt, 50.0f, GIMBAL_MAX_CURRENT);
    }

    // 云台零点
    DJIMotor_SetZero(SMALL_YAW_ZERO, SMALLYAWMotor);
    
    // 初始化校准变量
    Gimbal.pitch.inited = 0;
    Gimbal.pitch.offset = 0.0f;

    // 重力积分补偿器初始化
    Gimbal.pitch.grav_integral = 0.0f;
    Gimbal.pitch.ki_grav = 0.1f;          // 起步值，可在线调整
    Gimbal.pitch.grav_int_limit = 0.8f;   // N·m 限幅

    // 大yaw前馈
    Gimbal.yaw.big_ff = 0;
}


/* ============== 内部控制流程 ============== */
namespace {

/**
 * @brief  控制量更新（包括状态量和目标量）
 * @note   速度使用IMU解耦投影，位置使用连续累积角度避免±180°跳变
 */
void Update()
{
    /*------状态量更新------*/
    // 速度: 将IMU三轴角速度投影到yaw轴 (消除pitch姿态耦合)
    Gimbal.yaw.spd_raw = cos(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[2])
                        - sin(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[0]);
    // 一阶低通滤波: y = alpha * x + (1 - alpha) * y_last
    Gimbal.yaw.spd_now = Gimbal.yaw.lpf_alpha * Gimbal.yaw.spd_raw
                        + (1.0f - Gimbal.yaw.lpf_alpha) * Gimbal.yaw.spd_now;
    Gimbal.pitch.spd_now = -imu_gimbal.gyro[1];

    // 位置
    Gimbal.yaw.pos_now   = imu_gimbal.yaw_cnt;  // 使用连续累积角度
    Gimbal.pitch.pos_now  = degree2rad(-imu_gimbal.pitch);

    /*------上电初始化校准 (计算IMU和电机坐标系的差值)------*/
    if (Gimbal.pitch.inited == 0)
    {
        DM_motor_data_s dm = DMMotor_GetData(PITCHMotor);
        if (dm.motor_data.para.id != 0)  // 检查电机是否在线
        {
            Gimbal.pitch.offset = dm.motor_data.para.pos - Gimbal.pitch.pos_now;
            Global.Gimbal.input.pitch = -imu_gimbal.pitch;
            Gimbal.pitch.inited = 1;
        }
    }

    /*------目标量更新------*/
    Gimbal.pitch.pos_set = degree2rad(Global.Gimbal.input.pitch);

    if (AutoAim_IsControllingGimbal())
    {
        if (AutoAim_IsScanning())
            Gimbal.yaw.pos_set = Gimbal.yaw.pos_now;  // 扫描态: 位置跟随, 防退出时跳变
        else
            Gimbal.yaw.pos_set = Global.Gimbal.input.yaw;  // 跟踪态
    }
    else
    {
        // 手动: 遥控/导航
        if (Global.Chassis.mode == FLOW_Gimbal)
        {
            float enc_rel = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
            Gimbal.yaw.pos_set = Gimbal.yaw.pos_now - enc_rel;
        }
        else
        {
            Gimbal.yaw.pos_set = Global.Gimbal.input.yaw;
        }
    }
}


/**
 * @brief  控制量解算 (小Yaw)
 * @note   支持 LQR/LQI 和 PID串级 两条路径, 由 SMALL_YAW_USE_LQR 宏切换
 */
void Calculate()
{
#if SMALL_YAW_USE_LQR
    /* ===== LQR/LQI 路径 ===== */
    if (!AutoAim_IsTracking())
    {
        if (AutoAim_IsScanning())
        {
            // 扫描态: 用编码器速度做反馈, 避免IMU世界系耦合导致抖动
            float ff = GetBigYawFF_DegS();
            float cmd = Gimbal.yaw.spd_set + ff;
            float enc_spd = (float)DJIMotor_GetData(SMALLYAWMotor).speed_rpm * 6.0f;
            Gimbal.yaw.current = PID_Cal(&Gimbal.yaw.spd_pid, enc_spd, cmd);
        }
        else
        {
            // 手动/导航: LQR 状态反馈, 大yaw前馈作为速度参考
            float ff = GetBigYawFF_DegS();
            Gimbal.yaw.current = LQR_Calc(&Gimbal.yaw.lqr,
                Gimbal.yaw.pos_now, Gimbal.yaw.spd_now,
                Gimbal.yaw.pos_set, ff, 0.0f);
        }
    }
    else
    {
        // 跟踪态: LQR + 视觉前馈
        float yaw_ff = rad2degree(Global.Auto.input.yaw_ff);
        if (fabsf(yaw_ff) > aa_yaw_ff_max_deg_s) yaw_ff = 0.0f;
        float rate_ref = yaw_ff + GetBigYawFF_DegS();

        Gimbal.yaw.current = LQR_Calc(&Gimbal.yaw.lqr_auto,
            Gimbal.yaw.pos_now, Gimbal.yaw.spd_now,
            Gimbal.yaw.pos_set, rate_ref, 0.0f);
    }

#else
    /* ===== PID 串级路径 (回退通道) ===== */
    if (!AutoAim_IsTracking())
    {
        if (AutoAim_IsScanning())
        {
            // 扫描态: 编码器速度反馈, 与大yaw解耦
            float ff = GetBigYawFF_DegS();
            float cmd = Gimbal.yaw.spd_set + ff;
            float enc_spd = (float)DJIMotor_GetData(SMALLYAWMotor).speed_rpm * 6.0f;
            Gimbal.yaw.current = PID_Cal(&Gimbal.yaw.spd_pid, enc_spd, cmd);
        }
        else
        {
            // 非跟踪非扫描: 手动位置控制
            Gimbal.yaw.spd_set = PID_Cal(&Gimbal.yaw.pos_pid, Gimbal.yaw.pos_now, Gimbal.yaw.pos_set);
            Gimbal.yaw.current = PID_Cal(&Gimbal.yaw.spd_pid, Gimbal.yaw.spd_now, Gimbal.yaw.spd_set);
        }
    }
    else
    { 
        // 跟踪态: 视觉前馈 + 大yaw前馈 注入串级外环
        float yaw_ff = rad2degree(Global.Auto.input.yaw_ff);
        if (fabsf(yaw_ff) > aa_yaw_ff_max_deg_s) yaw_ff = 0.0f;

        Gimbal.yaw.spd_set = PID_Cal(&Gimbal.yaw.auto_pos_pid,
                                      Gimbal.yaw.pos_now, Gimbal.yaw.pos_set)
                            + yaw_ff + GetBigYawFF_DegS();

        Gimbal.yaw.current = PID_Cal(&Gimbal.yaw.auto_spd_pid,
                                      Gimbal.yaw.spd_now, Gimbal.yaw.spd_set);
    }
#endif
}


/**
 * @brief  电流值设置 / 电机输出
 * @note   处理模式切换瞬间的防跳变逻辑, 重力补偿, MIT/LQR控制输出
 */
void Control()
{
    static uint8_t last_mode = LOCK;
    static uint8_t last_chassis_mode = FLOW_Chassis;
    static uint8_t first_run = 1;

    // 第一帧初始化 last_mode
    if (first_run)
    {
        last_mode = Global.Control.mode;
        last_chassis_mode = Global.Chassis.mode;
        first_run = 0;
    }

    if (Global.Control.mode != LOCK)
    {
        // 判断小yaw是否需要泄力 (FLOW_Chassis无自瞄 / FLOW_Gimbal 都泄力)
        uint8_t yaw_release =
            ((Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode == NONE) ||
             (Global.Chassis.mode == FLOW_Gimbal));

        // 从FLOW_Chassis或FLOW_Gimbal切出的瞬间: 同步目标位置为当前位置, 防止跳变
        if ((last_chassis_mode == FLOW_Chassis && !yaw_release) ||
            (last_chassis_mode == FLOW_Gimbal && Global.Chassis.mode != FLOW_Gimbal))
        {
            Global.Gimbal.input.yaw = Gimbal.yaw.pos_now;
            Gimbal.yaw.pos_set = Gimbal.yaw.pos_now;
            // 清空速度环积分, 防止切模式时积累量导致冲击
            Gimbal.yaw.spd_pid.i_out = 0;
            Gimbal.yaw.auto_spd_pid.i_out = 0;
            LQR_Reset(&Gimbal.yaw.lqr);
            LQR_Reset(&Gimbal.yaw.lqr_auto);
        }

        // 重力补偿前馈力矩 (cos模型)
        Gimbal.pitch.grav_comp = Gimbal.pitch.k_gravity * cos(Gimbal.pitch.pos_now);

        if (Gimbal.pitch.inited == 1)
        {
             // 从LOCK模式切换出来的瞬间: 同步目标值, 防止跳变
             if (last_mode == LOCK)
             {
                Global.Gimbal.input.pitch = -imu_gimbal.pitch;
                Gimbal.pitch.pos_set = degree2rad(Global.Gimbal.input.pitch);
                Global.Gimbal.input.yaw = imu_gimbal.yaw_cnt;
                Gimbal.yaw.pos_set = Global.Gimbal.input.yaw;
                // LOCK→解锁瞬间清零积分, 防止历史积累造成冲击
                Gimbal.pitch.grav_integral = 0.0f;
                LQR_Reset(&Gimbal.yaw.lqr);
                LQR_Reset(&Gimbal.yaw.lqr_auto);
             }

             // 积分力矩补偿: 缓慢消除 k_gravity 标定不准 / 静摩擦 导致的稳态偏差
             float err = Gimbal.pitch.pos_set - Gimbal.pitch.pos_now;
             Gimbal.pitch.grav_integral += Gimbal.pitch.ki_grav * err * kCtrlDt;
             Gimbal.pitch.grav_integral = Clamp(Gimbal.pitch.grav_integral,
                                                -Gimbal.pitch.grav_int_limit,
                                                 Gimbal.pitch.grav_int_limit);

             // 总前馈力矩 = cos模型 + 积分修正
             float torque_ff = Gimbal.pitch.grav_comp + Gimbal.pitch.grav_integral;
             // 目标位置加上Offset, 转换到电机坐标系
             float target_motor = Gimbal.pitch.pos_set + Gimbal.pitch.offset;
             
             // 自瞄时注入速度前馈
             float vel_ff = 0;
             if (AutoAim_IsTracking())
                 vel_ff = Global.Auto.input.pitch_ff;

#if PITCH_USE_LQR
             // LQR 外环: IMU角度/角速度状态反馈, 重力补偿作为 u_ff 注入
             float lqr_torque = LQR_Calc(&Gimbal.pitch.lqr,
                 Gimbal.pitch.pos_now, Gimbal.pitch.spd_now,
                 Gimbal.pitch.pos_set, vel_ff, torque_ff);

             DMMotor_Set(PITCHMotor, target_motor, 0.0f, lqr_torque, 0.0f, 0.0f);
#else
             // MIT 位置+速度+力矩混合控制
             DMMotor_Set(PITCHMotor, target_motor, vel_ff, torque_ff,
                         Gimbal.pitch.kp, Gimbal.pitch.kd);
#endif
        }
        else
        {
             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        // 小yaw: 泄力模式下不输出
        if (yaw_release)
            DJIMotor_Set(0, SMALLYAWMotor);
        else
            DJIMotor_Set(Gimbal.yaw.current, SMALLYAWMotor); 
    }
    else
    {
        DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0);
        DJIMotor_Set(0, SMALLYAWMotor);
    }
   
    last_mode = Global.Control.mode;
    last_chassis_mode = Global.Chassis.mode;
}

} // namespace (internal)

/*-------------------- Task --------------------*/

/**
 * @brief  云台任务 (1kHz 调用)
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
    Update();
    Calculate();
    Control();
#endif
}

/*-------------------- Set --------------------*/

/**
 * @brief 设置云台Pitch轴角度 (带限位)
 */
void Gimbal_SetPitchAngle(float angle)
{
    Global.Gimbal.input.pitch = Clamp(angle, (float)PITCHI_MIN_ANGLE, (float)PITCHI_MAX_ANGLE);
}

/**
 * @brief 设置云台Yaw轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

