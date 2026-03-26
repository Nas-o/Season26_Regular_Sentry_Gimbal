/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-26 18:45:39
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
#include "Gimbal.h"
#include "Global_status.h"

#include "User_math.h"
#include "remote_control.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "hipnuc_dec.h"
#include "motor.h"
#include "vofa+.h"
#include "Auto_control.h"

Gimbal_t Gimbal;
float relative_angle;

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

/**
 * @brief 扫描模式
 * @author Nas(1319621819@qq.com)
 * @note  在MIT模式下，通过平滑改变目标位置（Setpoint）来模拟速度控制
 */
void Scan() 
{
    const float PITCH_MAX = 15.0f;//PITCHI_MAX_ANGLE;    
    const float PITCH_MIN = PITCHI_MIN_ANGLE;   
    const float STEP = 0.15f;                    

    // 小 Yaw 扫描参数：相对大 Yaw 往复扫描，自动适配大 Yaw 任意旋转
    const float SCAN_LIMIT = 60.0f;   // ±60° 相对大Yaw（机械限位±60°，留5°安全余量）
    const float SCAN_STEP  = 0.18f;   // 180°/s

    static float scan_pitch_deg = 0.0f;
    static float scan_offset    = 0.0f;  // 期望的小Yaw相对大Yaw的偏移角度
    static int8_t yaw_dir       = -1;
    static uint8_t init_done    = 0;
    

    // 初始同步
    if (!init_done) {
        scan_pitch_deg = rad2degree(Gimbal.pitch.pitch_location_now);
        scan_offset = 0.0f;
        init_done = 1;
    }

    // 读取 IMU 实际角度 (度)
    float phys_now = rad2degree(Gimbal.pitch.pitch_location_now);

    if (Gimbal.pitch.turnover_pitch == 1) // 向上扫描
    {
        scan_pitch_deg += STEP;
        // 如果物理位置已经接近上限，或者目标值已经严重越界，则换向
        if (phys_now >= (PITCH_MAX - 1.0f) || scan_pitch_deg > (PITCH_MAX + 3.0f)) {
            Gimbal.pitch.turnover_pitch = 0;
        }
    }
    else // 向下扫描
    {
        scan_pitch_deg -= STEP;
        if (phys_now <= (PITCH_MIN + 1.0f) || scan_pitch_deg < (PITCH_MIN - 3.0f)) {
            Gimbal.pitch.turnover_pitch = 1;
        }
    }

    // 限制 scan_pitch_deg 范围，防止目标值无限累加
    if (scan_pitch_deg > PITCH_MAX + 3.0f) scan_pitch_deg = PITCH_MAX + 3.0f;
    if (scan_pitch_deg < PITCH_MIN - 3.0f) scan_pitch_deg = PITCH_MIN - 3.0f;

    // 同步给全局变量，防止被 Updater 覆盖
    Global.Gimbal.input.pitch = scan_pitch_deg;
    Gimbal.pitch.pitch_location_set = degree2rad(scan_pitch_deg);

    // --- 小 Yaw 轴：相对大 Yaw 往复扫描 ---
    // scan_offset 在 ±SCAN_LIMIT 内振荡
    scan_offset += yaw_dir * SCAN_STEP;
    if (scan_offset >= SCAN_LIMIT)  
    { 
        scan_offset = SCAN_LIMIT;  
        yaw_dir = -1; 
    }
    if (scan_offset <= -SCAN_LIMIT) 
    { 
        scan_offset = -SCAN_LIMIT; 
        yaw_dir =  1; 
    }

    // 通过编码器获取小Yaw相对大Yaw的实际角度，反推大Yaw的世界系角度
    // encoder_relative: 小Yaw相对大Yaw的物理角度
    float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    // big_yaw_world = 小Yaw世界角(IMU) - 小Yaw相对大Yaw的偏移(编码器)
    float big_yaw_world = Gimbal.small_yaw.small_yaw_location_now - encoder_relative;

    // 目标 = 大Yaw当前世界角 + 期望的相对偏移
    // 等效于 PID 误差 = scan_offset - encoder_relative，直接控制相对角度
    Gimbal.small_yaw.small_yaw_location_set = big_yaw_world + scan_offset;
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
    Gimbal.pitch.kp = 15.0f;  // 位置比例增益 
    Gimbal.pitch.kd = 2.0f;   // 速度阻尼增益
    Gimbal.pitch.k_gravity = 0.4;//0.891800784f; // 重力前馈系数
    
    /*PID速度环初始化*/ 
    // 遥控
    PID_Set(&Gimbal.pitch.pitch_speed_pid, 0.165f, 0.0f, 0.2f, 0.0f, 7, 0.1);
    PID_Set(&Gimbal.small_yaw.small_yaw_speed_pid, 50.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    PID_Set(&Gimbal.pitch.pitch_auto_speed_pid, 0.165f, 0, 0.2, 0.0f, 7, 0.1);
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_speed_pid, 78.0f, 0.0f, 3.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.pitch.pitch_location_pid, 16.0f, 0.0f, 10.0f, 0.0f, 12.57, 1000);
    PID_Set(&Gimbal.small_yaw.small_yaw_location_pid, 4.0f, 0.0f, 1.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    PID_Set(&Gimbal.pitch.pitch_auto_location_pid, 16.0f, 0.0f, 10, 0.0f, 12.57, 100);
    PID_Set(&Gimbal.small_yaw.small_yaw_auto_location_pid, 12.5f, 0.0f, 3.5f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);

    // 云台零点初始化
    DJIMotor_SetZero(SMALL_YAW_ZERO,SMALLYAWMotor);
    
    // 初始化校准变量
    Gimbal.pitch.init_flag = 0;
    Gimbal.pitch.pitch_offset = 0.0f;
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
    Gimbal.small_yaw.small_yaw_speed_now = (cos(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[2]) - sin(degree2rad(imu_gimbal.pitch)) * rad2degree(imu_gimbal.gyro[0]));
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
    if (!(Global.Auto.mode != NONE && Global.Auto.input.Auto_control_online > 0 && Global.Auto.input.fire == -1))
    {
        Gimbal.small_yaw.small_yaw_location_set = Global.Gimbal.input.yaw; 
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

    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        // 非自瞄 
        Gimbal.small_yaw.small_yaw_speed_set =  PID_Cal(&Gimbal.small_yaw.small_yaw_location_pid, Gimbal.small_yaw.small_yaw_location_now,Gimbal.small_yaw.small_yaw_location_set);  
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_speed_pid, Gimbal.small_yaw.small_yaw_speed_now, Gimbal.small_yaw.small_yaw_speed_set); 
        
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { 
        Gimbal.small_yaw.small_yaw_speed_set = PID_Cal(&Gimbal.small_yaw.small_yaw_auto_location_pid, Gimbal.small_yaw.small_yaw_location_now, Gimbal.small_yaw.small_yaw_location_set)
                                              + Global.Auto.input.yaw_ff;
        Gimbal.small_yaw.current = PID_Cal(&Gimbal.small_yaw.small_yaw_auto_speed_pid, Gimbal.small_yaw.small_yaw_speed_now, Gimbal.small_yaw.small_yaw_speed_set);
        
        Global.Auto.input.Auto_control_online--;
    }
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
    static uint8_t first_run = 1;

    // 第一帧初始化 last_mode
    if (first_run)
    {
        last_mode = Global.Control.mode;
        first_run = 0;
    }

    if (Global.Control.mode != LOCK)
   {
        // 重力补偿前馈力矩计算
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
             }

             // 目标位置需要加上Offset，转换到电机坐标系
             float pitch_target_motor = Gimbal.pitch.pitch_location_set + Gimbal.pitch.pitch_offset;
             
             // Pitch: DM电机MIT模式, 自瞄时注入速度前馈
             float pitch_vel_ff = 0;
             if (Global.Auto.input.fire != -1 && Global.Auto.input.Auto_control_online > 0)
                 pitch_vel_ff = Global.Auto.input.pitch_ff;
             
             DMMotor_Set(PITCHMotor, 
                         pitch_target_motor,                // 目标位置 
                         pitch_vel_ff,                      // 目标速度(前馈)
                         Gimbal.pitch.gravity_compensation , // 前馈力矩 (重力补偿)
                         Gimbal.pitch.kp,                   // 位置比例增益
                         Gimbal.pitch.kd);                  // 速度阻尼增益
        }
        else
        {

             DMMotor_Set(PITCHMotor, 0, 0, 0, 0, 0.0f); 
        }
        
        DJIMotor_Set(Gimbal.small_yaw.current,SMALLYAWMotor); 
   }
   else
   {
        DMMotor_Set(PITCHMotor,0,0,0,0,0);
        DJIMotor_Set(0,SMALLYAWMotor);
   }
   
   last_mode = Global.Control.mode;

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

  if (Global.Auto.mode != NONE && 
        Global.Auto.input.Auto_control_online > 0 && 
        Global.Auto.input.fire == -1)
    {
        Scan();
        Auto_data.is_scaning = 1;
    }
    else
    {
        Auto_data.is_scaning = 0;
    }

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





