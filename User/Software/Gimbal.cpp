/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-05 19:55:59
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Gimbal.cpp
 * @brief  云台控制器 —— C++类实现: 初始化、数据更新、控制解算、电机输出
*/

#include "Gimbal.hpp"          
#include "Global_status.hpp"  

#include "User_math.hpp"     
#include "remote_control.hpp"
#include "IMU_updata.hpp"     
#include "dm_imu.hpp"       
#include "motor.hpp"        
#include "vofa+.hpp"           
#include "Auto_control.hpp"    

using namespace robot::gimbal;

GimbalController Gimbal;          
volatile float relative_angle;     

/**
 * @brief  云台角度限位
 * @param  pitch_up    Pitch轴向上机械极限角度 (rad)
 * @param  pitch_down  Pitch轴向下机械极限角度 (rad)
 * @param  yaw_l      Yaw轴左极限
 * @param  yaw_r      Yaw轴右极限 
 * @note   仅在目标角超出限位且运动方向朝向极限时才钳位,
 *         避免从极限位置回中时被错误限制
 */
void GimbalController::limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r)
{
    // 目标超过上限 且 当前位置大于目标 → 防止继续向上
    if (pitch.pos_set <= pitch_up && pitch.pos_set < pitch.pos_now)
        pitch.pos_set = pitch_up;
    // 目标超过下限 且 当前位置小于目标 → 防止继续向下
    else if (pitch.pos_set >= pitch_down && pitch.pos_set > pitch.pos_now)
        pitch.pos_set = pitch_down;
}

/*-------------------- Private: getBigYawFF_DegS --------------------*/

/**
 * @brief  获取大Yaw轴角速度前馈 (单位: deg/s)
 * @return 大Yaw前馈值, 用于补偿底盘旋转对小Yaw的扰动
 * @note   - 当编码器数据不新鲜(通信丢失)时返回0, 避免使用过期数据
 *         - 前馈值 = -(大yaw角速度 × 前馈增益)
 *         - 若前馈绝对值超过阈值 aa_big_yaw_ff_max_deg_s, 则视为异常返回0
 *         - 负号表示方向补偿: 底盘顺时针转 → 小yaw需逆时针补偿
 */
float GimbalController::getBigYawFF_DegS()
{
    // 编码器数据超时 → 不做前馈补偿
    if (!RelativeAngle_IsFresh())
        return 0.0f;

    // 计算前馈: 大yaw实际角速度 × 增益, 取反方向
    float ff = -yaw.big_ff * big_yaw_ff_gain;
    // 超限保护: 防止通信抖动产生过大前馈
    if (fabsf(ff) > big_yaw_ff_max_deg_s)
        return 0.0f;

    return ff;
}

/*-------------------- Public: init --------------------*/

/**
 * @brief  云台控制器初始化
 * @note   在RTOS启动前由 Gimbal_Init() C包装调用一次
 *         完成以下工作:
 *         1. Pitch轴 DM4310 电机初始化 (MIT模式)
 *         2. 小Yaw轴 GM6020 电机初始化
 *         3. Pitch MIT模式 Kp/Kd/重力补偿系数
 *         4. 小Yaw PID 串级参数 (位置环+速度环)
 *         5. 小Yaw 编码器零点标定
 *         6. Pitch 校准标志位及重力积分补偿器归零
 */
void GimbalController::init()
{
    /* ---- 1. 电机驱动初始化 ---- */
    DMMotor_Init(DM_4310, PITCHMotor);      
    DJIMotor_Init(DJI_GM6020, SMALLYAWMotor);
    
    /* ---- 2. Pitch MIT模式参数 ---- */
    pitch.kp = 18.0f;
    pitch.kd = 1.2f;
    // k_gravity: 静态重力力矩系数, 用于 τ = k*cos(θ) 前馈补偿
    pitch.k_gravity = 0.891800784f;

    /* ---- 3. 小Yaw PID 串级初始化 ---- */

    PID_Set(&yaw.spd_pid,      65.0f, 0.01f, 3.0f, 0.0f, GIMBAL_MAX_CURRENT, GIMBAL_MAX_CURRENT);
    PID_Set(&yaw.auto_spd_pid, 85.0f, 0.01f, 3.0f, 0.0f, GIMBAL_MAX_CURRENT, GIMBAL_MAX_CURRENT);
    PID_Set(&yaw.pos_pid,      18.0f, 0.0f,  1.0f, 0.0f, GIMBAL_MAX_CURRENT, 1000);
    PID_Set(&yaw.auto_pos_pid, 18.0f, 0.0f,  1.0f, 0.0f, GIMBAL_MAX_CURRENT, 1000);

    // 小yaw速度一阶低通滤波: alpha=1.0 表示不滤波(直通)
    yaw.lpf_alpha = 1.0f;
    yaw.spd_now = 0.0f;
    yaw.spd_raw = 0.0f;

    /* ---- 4. 小Yaw 编码器零点 ---- */
    // 设置GM6020编码器零偏, 使相对角度=0对应云台正前方
    DJIMotor_SetZero(SMALL_YAW_ZERO, SMALLYAWMotor);
    
    /* ---- 5. Pitch 校准变量 ---- */
    // inited=0: 等待DM4310首包反馈后完成零位标定
    pitch.inited = 0;
    pitch.offset = 0.0f;

    /* ---- 6. 重力积分补偿器 ---- */
    // 用积分消除静态重力补偿的残差 (k_gravity标定不精确时)
    pitch.grav_integral = 0.0f;   // 积分累计值
    pitch.ki_grav = 0.1f;         // 积分增益
    pitch.grav_int_limit = 0.8f;  // 积分限幅 (Nm)

    /* ---- 7. 大Yaw前馈 ---- */
    // 由CAN接收大yaw电机角速度, 用于小yaw前馈补偿
    yaw.big_ff = 0;
}

/*-------------------- Private: update --------------------*/

/**
 * @brief  更新云台状态量和目标量
 * @note   每个控制周期(1kHz)调用一次, 完成:
 *         1. 从IMU读取当前角度/角速度
 *         2. Pitch首次上电零位标定
 *         3. 根据模式计算Yaw/Pitch目标值
 */
void GimbalController::update()
{
    /*=== 1. 状态量更新: 从IMU获取当前角度和角速度 ===*/
    
    // Yaw角速度: 将IMU body系角速度投影到水平面
    // 公式: ωyaw = cos(pitch)*ωz - sin(pitch)*ωx
    // 原因: IMU固连在Pitch轴上, 当Pitch倾斜时z轴不再垂直
    yaw.spd_raw = cos(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[2])
                - sin(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[0]);
    // 一阶低通滤波: y[n] = α*x[n] + (1-α)*y[n-1]
    yaw.spd_now = yaw.lpf_alpha * yaw.spd_raw
                + (1.0f - yaw.lpf_alpha) * yaw.spd_now;
    yaw.spd_abs = yaw.spd_now;
    // Pitch角速度: 直接取IMU Y轴陀螺仪 (取反匹配坐标系)
    pitch.spd_now = -imu_gimbal.gyro[1];

    // Yaw位置: 使用IMU连续计数值(不归零, 避免±180°跳变)
    yaw.pos_abs   = imu_gimbal.yaw_cnt;
    yaw.pos_now   = yaw.pos_abs;
    DJI_motor_data_s small_yaw = DJIMotor_GetData(SMALLYAWMotor);
    yaw.pos_rel   = (float)small_yaw.angle;
    yaw.spd_rel   = (float)small_yaw.speed_rpm * 6.0f;
    // Pitch位置: 从度转弧度, 取反匹配电机正方向
    pitch.pos_now = degree2rad(-imu_gimbal.pitch);

    /*=== 2. 上电初始化校准 (仅执行一次) ===*/
    if (pitch.inited == 0)
    {
        // 等待DM4310电机返回第一帧有效数据 (id≠0)
        DM_motor_data_s dm = DMMotor_GetData(PITCHMotor);
        if (dm.motor_data.para.id != 0)
        {
            // offset = 电机绝对位置 - IMU当前角度
            // 之后控制时: 电机目标 = IMU目标 + offset
            pitch.offset = dm.motor_data.para.pos - pitch.pos_now;
            // 将当前Pitch角度设为初始目标(防止上电突跳)
            Global.Gimbal.input.pitch = -imu_gimbal.pitch;
            pitch.inited = 1;
        }
    }

    /*=== 3. 目标量更新 ===*/
    // Pitch目标: 从全局输入(度)转弧度
    pitch.pos_set = degree2rad(Global.Gimbal.input.pitch);

    // Yaw目标: 根据自瞄/底盘模式分支
    if (AutoAim_IsControllingGimbal())
    {
        if (AutoAim_IsScanning())
            // 扫描模式: 保持当前朝向, 速度环接管
            yaw.pos_set = yaw.pos_now;
        else
            // 自瞄跟踪: 目标由视觉计算写入 Global.Gimbal.input.yaw
            yaw.pos_set = Global.Gimbal.input.yaw;
    }
    else
    {
        if (Global.Chassis.mode == FLOW_Gimbal)
        {
            // 底盘跟随云台模式: 小yaw需用编码器补偿底盘转动
            // enc_rel: 小yaw相对底盘的偏移角
            float enc_rel = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
            // IMU目标 = 当前IMU角 - 编码器偏移 → 保持云台绝对朝向不变
            yaw.pos_set = yaw.pos_now - enc_rel;
        }
        else
        {
            // 其他模式: 直接使用遥控器/键鼠设定的目标角
            yaw.pos_set = Global.Gimbal.input.yaw;
        }
    }
}

/*-------------------- Private: calculate --------------------*/

/**
 * @brief  小Yaw轴控制量解算
 * @note   使用PID控制，分三种场景:
 *         A) 自瞄扫描中 (无目标): 纯速度环跟踪扫描速度
 *         B) 手动/非跟踪: 位置+速度串级
 *         C) 自瞄跟踪中: 使用自瞄参数 + 视觉速度前馈
 *
 *         输出: yaw.current → GM6020电流指令 (±GIMBAL_MAX_CURRENT)
 */
void GimbalController::calculate()
{
    if (!AutoAim_IsTracking())
    {
        if (AutoAim_IsScanning())
        {
            // 场景A: 扫描模式 → 纯速度环
            float ff = getBigYawFF_DegS();
            float cmd = yaw.spd_set + ff;
            yaw.current = PID_Cal(&yaw.spd_pid, yaw.spd_rel, cmd);
        }
        else
        {
            // 场景B: 手动模式 → 位置环输出速度指令 → 速度环输出电流
            yaw.spd_set = PID_Cal(&yaw.pos_pid, yaw.pos_now, yaw.pos_set);
            yaw.current = PID_Cal(&yaw.spd_pid, yaw.spd_now, yaw.spd_set);
        }
    }
    else
    { 
        // 场景C: 自瞄跟踪 → 自瞄位置环 + 视觉前馈 → 自瞄速度环
        float yaw_ff = rad2degree(Global.Auto.input.yaw_ff);
        if (fabsf(yaw_ff) > yaw_ff_max_deg_s) yaw_ff = 0.0f;

        // 速度指令 = 位置环输出 + 视觉角速度前馈 + 大Yaw前馈
        yaw.spd_set = PID_Cal(&yaw.auto_pos_pid,
                              yaw.pos_now, yaw.pos_set)
                    + yaw_ff + getBigYawFF_DegS();

        yaw.current = PID_Cal(&yaw.auto_spd_pid,
                              yaw.spd_now, yaw.spd_set);
    }
}

/*-------------------- Private: control --------------------*/

/**
 * @brief  云台电机输出控制
 * @note   根据全局模式决定是否输出力矩/电流:
 *         - LOCK模式: 所有电机零输出 (云台自由落体)
 *         - 非LOCK:
 *           * Pitch: MIT力矩模式, 含重力补偿+积分修正
 *           * Yaw:   GM6020电流模式, 含模式切换防突跳处理
 *
 *         模式切换防突跳逻辑:
 *         - LOCK→非LOCK: 将目标对齐当前值, 清零积分
 *         - 底盘模式变化: 重置Yaw目标防止累积误差导致甩头
 */
void GimbalController::control()
{
    // 首次运行: 记录初始模式, 避免首帧误判为"模式切换"
    if (first_run_)
    {
        last_mode_ = Global.Control.mode;
        last_chassis_mode_ = Global.Chassis.mode;
        first_run_ = 0;
    }

    if (Global.Control.mode != LOCK)
    {
        /* ---- Yaw释放条件判断 ---- */
        // yaw_release=1: 小yaw不输出力矩, 让底盘自由跟随
        // 条件: (底盘跟随且无自瞄) 或 (云台跟随模式)
        uint8_t yaw_release =
            ((Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode == NONE) ||
             (Global.Chassis.mode == FLOW_Gimbal));

        /* ---- 底盘模式切换 → 重置Yaw ---- */
        // 从底盘跟随切到非释放 / 从云台跟随切出: 清零积分, 对齐目标
        if ((last_chassis_mode_ == FLOW_Chassis && !yaw_release) ||
            (last_chassis_mode_ == FLOW_Gimbal && Global.Chassis.mode != FLOW_Gimbal))
        {
            Global.Gimbal.input.yaw = yaw.pos_now;
            yaw.pos_set = yaw.pos_now;
            yaw.spd_pid.i_out = 0;
            yaw.auto_spd_pid.i_out = 0;
        }

        /* ---- Pitch重力补偿 ---- */
        // 静态力矩: τ = k × cos(θ), θ为当前Pitch角(rad)
        pitch.grav_comp = pitch.k_gravity * cos(pitch.pos_now);

        if (pitch.inited == 1)
        {
             /* ---- LOCK→解锁 过渡处理 ---- */
             if (last_mode_ == LOCK)
             {
                // 将目标对齐当前, 防止解锁瞬间大角度突跳
                Global.Gimbal.input.pitch = -imu_gimbal.pitch;
                pitch.pos_set = degree2rad(Global.Gimbal.input.pitch);
                Global.Gimbal.input.yaw = imu_gimbal.yaw_cnt;
                yaw.pos_set = Global.Gimbal.input.yaw;
                // 清零积分补偿, 防止解锁时积累的积分引起过冲
                pitch.grav_integral = 0.0f;
                PID_Reset(&yaw.spd_pid, yaw.spd_now, 0.0f);
                PID_Reset(&yaw.auto_spd_pid, yaw.spd_now, 0.0f);
                PID_Reset(&yaw.pos_pid, yaw.pos_now, yaw.pos_now);
                PID_Reset(&yaw.auto_pos_pid, yaw.pos_now, yaw.pos_now);
             }

             /* ---- Pitch重力积分补偿 ---- */
             // 用位置误差积分修正重力补偿系数的标定偏差
             float err = pitch.pos_set - pitch.pos_now;
             pitch.grav_integral += pitch.ki_grav * err * kCtrlDt;
             pitch.grav_integral = clamp(pitch.grav_integral,
                                         -pitch.grav_int_limit,
                                          pitch.grav_int_limit);

             // 总前馈力矩 = 静态重力 + 积分修正
             float torque_ff = pitch.grav_comp + pitch.grav_integral;
             // 电机端目标位置 = IMU目标角(rad) + 零位偏移
             float target_motor = pitch.pos_set + pitch.offset;
             
             // 自瞄跟踪时叠加视觉Pitch速度前馈
             float vel_ff = 0;
             if (AutoAim_IsTracking())
                 vel_ff = Global.Auto.input.pitch_ff;

             /* ---- Pitch MIT-PD路径 ---- */
             // MIT模式: pos=target, vel=前馈, torque=重力补偿, kp/kd=刚度阻尼
             DMMotor_Set(PITCHMotor, target_motor, vel_ff, torque_ff,
                         pitch.kp, pitch.kd);
        }
        else
        {
             // Pitch未校准: 零输出等待首帧数据
             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        /* ---- 小Yaw输出 ---- */
        if (yaw_release)
            DJIMotor_Set(0, SMALLYAWMotor);         // 释放: 零电流
        else
            DJIMotor_Set(yaw.current, SMALLYAWMotor); // 正常: 输出解算电流
    }
    else
    {
        /* ---- LOCK模式: 全部零输出 ---- */
        DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0);
        DJIMotor_Set(0, SMALLYAWMotor);
    }
   
    // 记录本帧模式, 供下帧检测模式切换
    last_mode_ = Global.Control.mode;
    last_chassis_mode_ = Global.Chassis.mode;
}

/*-------------------- Public: task --------------------*/

/**
 * @brief  云台控制主任务入口 (1kHz)
 * @note   由FreeRTOS的Gimbal_Task每1ms调用一次
 *         执行顺序: update(状态更新) → calculate(解算) → control(输出)
 *         USE_GIMBAL=0 时整个云台控制被编译裁剪
 */
void GimbalController::task()
{
#if (USE_GIMBAL != 0)
    update();    
    calculate();   
    control();   
#endif
}

/*-------------------- Public: setPitchAngle --------------------*/

/**
 * @brief  设置Pitch目标角度 (带限幅保护)
 * @param  angle  目标角度 (度), 正值抬头, 负值低头
 * @note   限幅范围: [PITCHI_MIN_ANGLE, PITCHI_MAX_ANGLE]
 *         由遥控器解析或自瞄模块调用
 */
void GimbalController::setPitchAngle(float angle)
{
    Global.Gimbal.input.pitch = clamp(angle, (float)PITCHI_MIN_ANGLE, (float)PITCHI_MAX_ANGLE);
}

/*-------------------- Public: setYawAngle --------------------*/

/**
 * @brief  设置Yaw目标角度 (无限幅, 连续累计)
 * @param  angle  目标角度 (度), 使用IMU连续计数坐标系
 * @note   不做限幅: Yaw轴可无限旋转 (连续计数不归零)
 *         由遥控器解析或自瞄模块调用
 */
void GimbalController::setYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

/* ============================================================
 *  C兼容包装函数 (供FreeRTOS任务 / C源文件调用)
 *  由于 freertos.c 等文件以C编译, 无法直接调用C++成员函数,
 *  因此提供以下 extern "C" 包装, 内部转发给类实例.
 * ============================================================ */

void Gimbal_Init()          { Gimbal.init(); }
void Gimbal_Tasks()         { Gimbal.task(); }
void Gimbal_SetPitchAngle(float angle) { Gimbal.setPitchAngle(angle); }
void Gimbal_SetYawAngle(float angle)   { Gimbal.setYawAngle(angle); }
void Gimbal_Limit(float pitch_up, float pitch_down, float yaw_l, float yaw_r)
{
    Gimbal.limit(pitch_up, pitch_down, yaw_l, yaw_r);
}

