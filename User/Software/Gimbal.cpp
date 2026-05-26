/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-26 21:24:26
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Gimbal.c
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

void Gimbal_Limit(float pitch_up_angle, float pitch_down_angle, float yaw_L_angle, float yaw_R_angle)
{
    if (Gimbal.pitch.pitch_location_set <= pitch_up_angle && Gimbal.pitch.pitch_location_set < Gimbal.pitch.pitch_location_now)
    {
        Gimbal.pitch.pitch_location_set = pitch_up_angle;
    }
    else if (Gimbal.pitch.pitch_location_set >= pitch_down_angle && Gimbal.pitch.pitch_location_set > Gimbal.pitch.pitch_location_now)
    {
        Gimbal.pitch.pitch_location_set = pitch_down_angle;
    }
}

static float Gimbal_GetSmallYawBigFollowFFDegS(void)
{
    float ff_deg_s = 0.0f;

    /* 对板数据不新鲜时，绝不使用这路前馈 */
    if (!RelativeAngle_IsFresh())
        return 0.0f;

    /* 下板发来的是 deg/s，这里不要再做 rad2degree */
    ff_deg_s = -Gimbal.small_yaw.small_from_big_feedforward * aa_big_yaw_ff_gain;

    /* 执行安全限幅 */
    if (fabsf(ff_deg_s) > aa_big_yaw_ff_max_deg_s)
        return 0.0f;

    return ff_deg_s;
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化

    DMMotor_Init(DM_4310, PITCHMotor);
    DJIMotor_Init(DJI_GM6020, SMALLYAWMotor);
    
    /*------MIT模式参数初始化------*/
    Gimbal.pitch.kp = 18.0f;  // 位置比例增益 
    Gimbal.pitch.kd = 1.2f;   // 速度阻尼增益
    Gimbal.pitch.k_gravity = 0.891800784f; // 重力前馈系数

    /*------Pitch LQR 外环初始化 (PITCH_USE_LQR=1 时生效)------*/
    {
        /* 状态 = [pitch_angle_rad, pitch_rate_rad_s], 输出 = 力矩(N·m)
         * 暂用 2 态 LQR (不含积分, 因为已有 gravity_integral 做稳态修正)
         * 单位: rad, rad/s -> N·m, 注意这里和 yaw 的 deg 单位不同!
         * 占位值, 需 MATLAB 辨识后替换 */
        float K_pitch[2] = {
            18.0f,     // K_angle (rad -> N·m)
             1.2f      // K_rate  (rad/s -> N·m)
        };
        LQR_Init(&Gimbal.pitch.lqr, K_pitch, 2, 0.001f,
                 0.0f,    // 不使用积分 (dim=2)
                 3.0f);   // 输出限幅 N·m
    }
    
    /*PID速度环初始化*/ 
    // 遥控
    PID_Set(&Gimbal.small_yaw.small_yaw_speed_pid, 65.0f, 0.01f, 3.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_speed_pid, 85.0f, 0.01f, 3.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.small_yaw.small_yaw_location_pid, 18.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_location_pid, 18.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);

    // 小yaw速度一阶低通滤波系数 (0~1, 越小滤波越强)
    Gimbal.small_yaw.small_yaw_speed_lpf_alpha = 1.0f;
    Gimbal.small_yaw.small_yaw_speed_now = 0.0f;
    Gimbal.small_yaw.small_yaw_speed_raw = 0.0f;

    /*------小Yaw LQR/LQI 初始化------*/
    /*
     * K 增益由 MATLAB dlqr/lqr 离线求得, 状态 = [angle_deg, rate_deg_s, integral]
     * 这里给的是占位初值, 需要根据实际辨识的模型重新计算后替换!
     * 建议: 先在 MATLAB 中用简化模型 J*ddtheta = Kt*i 辨识 J, 然后
     *   A = [0 1; 0 0]; B = [0; Kt/J];
     *   增广积分后 A_aug = [A [0;0]; -1 0 0]; B_aug = [B; 0];
     *   Q = diag([q_angle, q_rate, q_int]); R = r_current;
     *   [K,~,~] = dlqr(Ad_aug, Bd_aug, Q, R);
     */
    {
        /* ===== 手动/导航模式 LQR 增益 ===== */
        float K_manual[3] = {
            120.0f,    // K_angle: 角度误差增益 (对应 deg -> 电流)
              8.0f,    // K_rate:  角速度误差增益 (对应 deg/s -> 电流)
             60.0f     // K_int:   积分增益 (消除稳态误差)
        };
        LQR_Init(&Gimbal.small_yaw.lqr, K_manual, 3, 0.001f,
                 50.0f,                    // 积分限幅 (deg*s)
                 GIMBALMOTOR_MAX_CURRENT); // 输出限幅

        /* ===== 自瞄模式 LQR 增益 (可独立调参, 通常更激进) ===== */
        float K_auto[3] = {
            150.0f,    // K_angle
             10.0f,    // K_rate
             80.0f     // K_int
        };
        LQR_Init(&Gimbal.small_yaw.lqr_auto, K_auto, 3, 0.001f,
                 50.0f,
                 GIMBALMOTOR_MAX_CURRENT);
    }

    // 云台零点初始化
    DJIMotor_SetZero(SMALL_YAW_ZERO,SMALLYAWMotor);
    
    // 初始化校准变量
    Gimbal.pitch.init_flag = 0;
    Gimbal.pitch.pitch_offset = 0.0f;

    // 重力积分补偿器初始化
    Gimbal.pitch.gravity_integral = 0.0f;
    Gimbal.pitch.ki_gravity = 0.1f;             // 起步值，可在线调整
    Gimbal.pitch.gravity_integral_limit = 0.8f; // N·m 限幅

    //大yaw前馈
    Gimbal.small_yaw.small_from_big_feedforward = 0;
}


/*-------------------- Update --------------------*/
 
/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none           
 */
void Gimbal_Updater()
{
    /*------状态量更新------*/
    //速度                                       
    Gimbal.small_yaw.small_yaw_speed_raw = (cos(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[2]) - sin(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[0]));
    // 一阶低通滤波: y = alpha * x + (1 - alpha) * y_last
    Gimbal.small_yaw.small_yaw_speed_now = Gimbal.small_yaw.small_yaw_speed_lpf_alpha * Gimbal.small_yaw.small_yaw_speed_raw
                                         + (1.0f - Gimbal.small_yaw.small_yaw_speed_lpf_alpha) * Gimbal.small_yaw.small_yaw_speed_now;
    Gimbal.pitch.pitch_speed_now = -imu_gimbal.gyro[1];
    //位置

    Gimbal.small_yaw.small_yaw_location_now = imu_gimbal.yaw_cnt; // 使用连续累积角度，避免±180°跳变
    Gimbal.pitch.pitch_location_now = degree2rad(-imu_gimbal.pitch);

    /*------上电初始化校准(计算IMU和电机坐标系的差值)------*/
    if (Gimbal.pitch.init_flag == 0)
    {
        // 检查电机是否在线 (ID非0表示已接收到至少一帧反馈数据)
        DM_motor_data_s pitch_motor_data = DMMotor_GetData(PITCHMotor);
        if (pitch_motor_data.motor_data.para.id != 0) 
        {
            float motor_pos = pitch_motor_data.motor_data.para.pos; // 电机位置(角度转弧度)
            Gimbal.pitch.pitch_offset = motor_pos - Gimbal.pitch.pitch_location_now; // offset = Motor - IMU
            
            // 同步目标位置
            Global.Gimbal.input.pitch = -imu_gimbal.pitch;
            
            Gimbal.pitch.init_flag = 1;
        }
    }

    /*------目标量更新------*/

    //位置

    Gimbal.pitch.pitch_location_set = degree2rad(Global.Gimbal.input.pitch);

    /* AutoAim_Update() 已在本拍写好 setpoint (跟踪/扫描/手动均由上层决定) */
    if (AutoAim_IsControllingGimbal())
    {
        if (AutoAim_IsScanning())
        {
            /* 扫描态: 速度由 Scan_Internal() 直接设置, 位置目标跟随当前值防止退出时跳变 */
            Gimbal.small_yaw.small_yaw_location_set = Gimbal.small_yaw.small_yaw_location_now;
        }
        else
        {
            /* 跟踪态 */
            Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw;
        }
    }
    else
    {
        /* 手动: 遥控/导航 */
    if (Global.Chassis.mode == FLOW_Gimbal)
        {
            float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
            float big_yaw_world = Gimbal.small_yaw.small_yaw_location_now - encoder_relative;
            Gimbal.small_yaw.small_yaw_location_set = big_yaw_world;
        }
        else
        {
            Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw; 
        }
    }
    // 假设 Gimbal.small_yaw.small_yaw_location_now 是当前小yaw的角度
/*     IMU_Rotate_Frame(Gimbal.small_yaw.small_yaw_location_now,&imu_gimbal); */
}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Gimbal_Calculater()
{
#if SMALL_YAW_USE_LQR
    /* ============ LQR/LQI 路径 ============ */
    if (!AutoAim_IsTracking())
    {
        if (AutoAim_IsScanning())
        {
            /* 扫描态保持 PID 速度环 (用编码器速度反馈, 与大yaw解耦) */
            float big_yaw_ff_deg_s = Gimbal_GetSmallYawBigFollowFFDegS();
            float scan_speed_cmd_deg_s = Gimbal.small_yaw.small_yaw_speed_set + big_yaw_ff_deg_s;
            float encoder_speed_deg_s = (float)DJIMotor_GetData(SMALLYAWMotor).speed_rpm * 6.0f;
            Gimbal.small_yaw.current =
                PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid,
                        encoder_speed_deg_s,
                        scan_speed_cmd_deg_s);
        }
        else
        {
            /* 手动/导航: LQR 状态反馈, 参考角速度 = 0 */
            float big_yaw_ff = Gimbal_GetSmallYawBigFollowFFDegS();
            Gimbal.small_yaw.current = LQR_Calc(
                &Gimbal.small_yaw.lqr,
                Gimbal.small_yaw.small_yaw_location_now,   // angle_now (deg)
                Gimbal.small_yaw.small_yaw_speed_now,      // rate_now  (deg/s)
                Gimbal.small_yaw.small_yaw_location_set,   // angle_ref (deg)
                big_yaw_ff,                                 // rate_ref  (deg/s) 大yaw前馈作为速度参考
                0.0f);                                      // u_ff
        }
    }
    else
    {
        /* 跟踪态: LQR + 视觉前馈 */
        float yaw_ff_deg_s = rad2degree(Global.Auto.input.yaw_ff);
        if (fabsf(yaw_ff_deg_s) > aa_yaw_ff_max_deg_s)
            yaw_ff_deg_s = 0.0f;

        float big_yaw_ff_deg_s = Gimbal_GetSmallYawBigFollowFFDegS();
        float rate_ref = yaw_ff_deg_s + big_yaw_ff_deg_s;

        Gimbal.small_yaw.current = LQR_Calc(
            &Gimbal.small_yaw.lqr_auto,
            Gimbal.small_yaw.small_yaw_location_now,
            Gimbal.small_yaw.small_yaw_speed_now,
            Gimbal.small_yaw.small_yaw_location_set,
            rate_ref,
            0.0f);
    }

#else
    /* ============ 原 PID 串级路径 (回退通道) ============ */
    if (!AutoAim_IsTracking())
    {
        if (AutoAim_IsScanning())
            {
                float big_yaw_ff_deg_s = Gimbal_GetSmallYawBigFollowFFDegS();
                float scan_speed_cmd_deg_s = Gimbal.small_yaw.small_yaw_speed_set + big_yaw_ff_deg_s;

                // 扫描态：用编码器速度做反馈，避免IMU世界系耦合导致抖动
                float encoder_speed_deg_s = (float)DJIMotor_GetData(SMALLYAWMotor).speed_rpm * 6.0f;
                Gimbal.small_yaw.current =
                    PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid,
                            encoder_speed_deg_s,
                            scan_speed_cmd_deg_s);
            }
        else
        {
            // 非跟踪非扫描：手动位置控制
            Gimbal.small_yaw.small_yaw_speed_set = PID_Cal(&Gimbal.small_yaw.small_yaw_location_pid, Gimbal.small_yaw.small_yaw_location_now, Gimbal.small_yaw.small_yaw_location_set);  
            Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid, Gimbal.small_yaw.small_yaw_speed_now, Gimbal.small_yaw.small_yaw_speed_set);
        }

    }
    else
    { 
        float yaw_ff_deg_s = 0.0f;
        float big_yaw_ff_deg_s = Gimbal_GetSmallYawBigFollowFFDegS();

        /* 视觉给的小yaw前馈: 原协议是 rad/s，这里转成 deg/s */
        yaw_ff_deg_s = rad2degree(Global.Auto.input.yaw_ff);
        if (fabsf(yaw_ff_deg_s) > aa_yaw_ff_max_deg_s)
            yaw_ff_deg_s = 0.0f;

        Gimbal.small_yaw.small_yaw_speed_set =
            PID_Cal(&Gimbal.small_yaw.small_yaw_auto_location_pid,
                    Gimbal.small_yaw.small_yaw_location_now,
                    Gimbal.small_yaw.small_yaw_location_set)
            + yaw_ff_deg_s
            + big_yaw_ff_deg_s;

        Gimbal.small_yaw.current =
            PID_Cal(&Gimbal.small_yaw.small_yaw_auto_speed_pid,
                    Gimbal.small_yaw.small_yaw_speed_now,
                    Gimbal.small_yaw.small_yaw_speed_set);
    }
#endif /* SMALL_YAW_USE_LQR */
}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Gimbal_Controller()
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
        uint8_t small_yaw_release =
            ((Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode == NONE) ||
             (Global.Chassis.mode == FLOW_Gimbal));

        // 从FLOW_Chassis或FLOW_Gimbal切出的瞬间：同步目标位置为当前位置，防止跳变
        if ((last_chassis_mode == FLOW_Chassis && !small_yaw_release) ||
            (last_chassis_mode == FLOW_Gimbal && Global.Chassis.mode != FLOW_Gimbal))
        {
            Global.Gimbal.input.yaw = Gimbal.small_yaw.small_yaw_location_now;
            Gimbal.small_yaw.small_yaw_location_set = Gimbal.small_yaw.small_yaw_location_now;
            // 清空速度环积分，防止切模式时积累量导致冲击
            Gimbal.small_yaw.small_yaw_speed_pid.i_out = 0;
            Gimbal.small_yaw.small_yaw_auto_speed_pid.i_out = 0;
            // 清零 LQR 积分
            LQR_Reset(&Gimbal.small_yaw.lqr);
            LQR_Reset(&Gimbal.small_yaw.lqr_auto);
        }

        // 重力补偿前馈力矩计算 (模型前馈 + 积分修正)
        Gimbal.pitch.gravity_compensation = Gimbal.pitch.k_gravity * cos(Gimbal.pitch.pitch_location_now);

        if (Gimbal.pitch.init_flag == 1)
        {
             // 从LOCK模式切换出来的瞬间，同步目标值为当前位置，防止跳变
             if (last_mode == LOCK)
             {
                Global.Gimbal.input.pitch = -imu_gimbal.pitch;
                Gimbal.pitch.pitch_location_set = degree2rad(Global.Gimbal.input.pitch);

                Global.Gimbal.input.yaw = imu_gimbal.yaw_cnt;
                Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw;

                // LOCK→解锁瞬间清零积分，防止历史积累造成冲击
                Gimbal.pitch.gravity_integral = 0.0f;
                LQR_Reset(&Gimbal.small_yaw.lqr);
                LQR_Reset(&Gimbal.small_yaw.lqr_auto);
             }

             // 积分力矩补偿：缓慢消除 k_gravity 标定不准 / 静摩擦 导致的稳态偏差
             float pitch_error = Gimbal.pitch.pitch_location_set - Gimbal.pitch.pitch_location_now;
             Gimbal.pitch.gravity_integral += Gimbal.pitch.ki_gravity * pitch_error * 0.001f; // 1ms 控制周期
             // 限幅
             if (Gimbal.pitch.gravity_integral > Gimbal.pitch.gravity_integral_limit)
                 Gimbal.pitch.gravity_integral = Gimbal.pitch.gravity_integral_limit;
             if (Gimbal.pitch.gravity_integral < -Gimbal.pitch.gravity_integral_limit)
                 Gimbal.pitch.gravity_integral = -Gimbal.pitch.gravity_integral_limit;

             // 总前馈力矩 = cos模型 + 积分修正
             float total_torque_ff = Gimbal.pitch.gravity_compensation + Gimbal.pitch.gravity_integral;

             // 目标位置需要加上Offset，转换到电机坐标系
             float pitch_target_motor = Gimbal.pitch.pitch_location_set + Gimbal.pitch.pitch_offset;
             
             // Pitch: DM电机MIT模式, 自瞄时注入速度前馈
             float pitch_vel_ff = 0;
             if (AutoAim_IsTracking())
                 pitch_vel_ff = Global.Auto.input.pitch_ff;

#if PITCH_USE_LQR
             /* LQR 外环: 用 IMU 角度/角速度做状态反馈, 输出力矩指令
              * 重力补偿作为 u_ff 注入, MIT 内层 kp=kd=0 只透传力矩 */
             float lqr_torque = LQR_Calc(
                 &Gimbal.pitch.lqr,
                 Gimbal.pitch.pitch_location_now,     // angle_now (rad)
                 Gimbal.pitch.pitch_speed_now,        // rate_now  (rad/s)
                 Gimbal.pitch.pitch_location_set,     // angle_ref (rad)
                 pitch_vel_ff,                        // rate_ref  (rad/s)
                 total_torque_ff);                    // u_ff (重力补偿)

             DMMotor_Set(PITCHMotor,
                         pitch_target_motor,          // pos (MIT 内部不用, kp=0)
                         0.0f,                        // vel (kd=0)
                         lqr_torque,                  // 力矩 = LQR输出 (已含重力补偿)
                         0.0f,                        // kp = 0 (纯力矩透传)
                         0.0f);                       // kd = 0
#else
             DMMotor_Set(PITCHMotor, 
                         pitch_target_motor,                // 目标位置 
                         pitch_vel_ff,                      // 目标速度(前馈)
                         total_torque_ff,                   // 前馈力矩 (重力模型 + 积分修正)
                         Gimbal.pitch.kp,                   // 位置比例增益
                         Gimbal.pitch.kd);                  // 速度阻尼增益
#endif
        }
        else
        {

             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        // 小yaw：FLOW_Chassis模式下泄力，其他模式正常输出
        if (small_yaw_release)
            DJIMotor_Set(0, SMALLYAWMotor);
        else
            DJIMotor_Set(Gimbal.small_yaw.current, SMALLYAWMotor); 
   }
   else
   {
        DMMotor_Set(PITCHMotor,0,0,0,0,0);
        DJIMotor_Set(0,SMALLYAWMotor);
   }
   
   last_mode = Global.Control.mode;
   last_chassis_mode = Global.Chassis.mode;

}

/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
	// 云台数据更新
	Gimbal_Updater();

    // 云台控制解算
    Gimbal_Calculater();
	// 云台电机控制
	Gimbal_Controller();
    
#endif
}


/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

