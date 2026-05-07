/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-07 17:25:17
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Chassis.c
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
#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "Auto_control.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"
#include "Shoot.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"
#include "dm_imu.h"

#include "User_math.h"
#include "robot_param.h"
#include "ramp_generator.h"


Chassis_t Chassis ;
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};   
RC_ctrl_t RC_data;
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
int Last_Hp = 400;
static uint8_t hurt_high_spin_latched = 0;
static uint8_t nav_offline_dash_active = 0;
static uint8_t nav_offline_dash_finished = 0;
static float nav_offline_dash_start_s = 0.0f;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{

    /*底盘跟随PID*/
    PID_Set(&Chassis.chassis_follow_pid, 0.60f, 0.0f, 0.0f, 0.0f, 120, 40);
    /*底盘功率控制pid*/


    //底盘运动斜坡
/*     RampGenerator_Init(&Chassis.Vx_ramp, CHASSIS_TASK_TIME, 40, 40, 2);
    RampGenerator_Init(&Chassis.Vy_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Chassis.Vw_ramp, CHASSIS_TASK_TIME, 300, 300, 4); */
    
    //默认地盘跟随模式
    Global.Chassis.mode = FLOW_Chassis;

    Chassis.random.refresh_interval = 250;
    srand(HAL_GetTick());

}

/*-------------------- AngleLimit --------------------*/

/**
 * @brief          多圈角度限制
 * @param          angle:要化简的角度
 * @retval         angle:化简后的角度
 * @author         Nas(1319621819@qq.com)
 */
float Chassis_AngleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;
    if (angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            angle -= (mul + 1) * 180.0f;
        else // 处于180度
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            angle += (mul + 1) * 180.0f;
        else // 处于-180度
            angle += mul * 180.0f;
    }
    return angle;
}

/*-----------------------obtain_modulus_normalization-----------------------*/
/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 * @author         Nas(1319621819@qq.com)
 */
float Obtain_Modulus_Normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}



/**************************** Random_Spin ****************************/

/**
 * @brief 任意转动
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Random_Spin(float min, float max)
{
    
    static int run_count = 0;//计数器，为0更方便看周期
	run_count++;
	float target_r;
	if (run_count >= Chassis.random.refresh_interval) // 判断周期更新
	{
		if (Chassis.random.smaller_than_2_count < 2) // 检查低于底线值的计数
		{
			Chassis.random.valve = Generate_Random_Float(min, max); // 生成随机值
			if (Chassis.random.valve < 7.0f) // 检查是否低于底线值
			{
				Chassis.random.smaller_than_2_count++;
			}
		}
		else // smaller_than_2_count >= 2，说明已经连续多次低于底线值了
		{
			Chassis.random.valve = 9.0f; // 返回固定值，强制提升速度
			Chassis.random.smaller_than_2_count = 0; // 重置标志位计数
		}
		run_count = 0; // 重置主计数器，开始新周期
	 }
	 return Chassis.random.valve;
}

/*********************** Generate_Random_Float ***********************/

/**
 * @brief 计算随机数
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Generate_Random_Float(float min, float max)
{
	return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

/********************* Variable_Spin_Update **********************/

/**
 * @brief  变速小陀螺 —— 交替高/低速段 + 斜坡平滑过渡
 *         每次 tick (1ms) 调用，返回平滑后的角速度 (rad/s)
 *
 *         原理：
 *         1) 转速分为 LOW [55,70] RPM 和 HIGH [90,110] RPM
 *         2) 到达目标后随机保持 300-800ms，再切换至对侧速段
 *         3) RampGenerator 做加/减速平滑，避免瞬间突变
 *         4) 交替策略保证最小速差 >= 40 RPM (≈4.2 rad/s)，变速肉眼可见
 *
 * @return float 当前角速度 (rad/s, 正方向)
 * @author Nas(1319621819@qq.com)
 */
float Variable_Spin_Update(void)
{
    /* ---- 可调参数 (RPM) ---- */
    #define VSPIN_LOW_MIN_RPM    60.0f     /* 低速段下限 RPM    */
    #define VSPIN_LOW_MAX_RPM    70.0f     /* 低速段上限 RPM    */
    #define VSPIN_HIGH_MIN_RPM   90.0f    /* 高速段下限 RPM    */
    #define VSPIN_HIGH_MAX_RPM   105.0f    /* 高速段上限 RPM    */
    #define VSPIN_HOLD_MIN_MS    300       /* 最短保持时间 ms   */
    #define VSPIN_HOLD_MAX_MS    800       /* 最长保持时间 ms   */

    static float    current_rad_s = 0.0f;
    static uint32_t hold_timer    = 0;
    static uint32_t hold_duration = 0;
    static uint8_t  is_high_phase = 0;     /* 0 = 低速段, 1 = 高速段 */
    static uint8_t  initialized   = 0;

    uint32_t now = HAL_GetTick();

    /* ---- 首次进入初始化 ---- */
    if (!initialized)
    {
        current_rad_s = Generate_Random_Float(VSPIN_LOW_MIN_RPM, VSPIN_LOW_MAX_RPM) * RPM_TO_RAD_S;
        hold_timer    = now;
        hold_duration = (uint32_t)Generate_Random_Float(VSPIN_HOLD_MIN_MS, VSPIN_HOLD_MAX_MS);
        is_high_phase = 0;
        initialized   = 1;
    }

    /* ---- 保持时间到期 => 切换速段，直接阶跃 ---- */
    if (now - hold_timer >= hold_duration)
    {
        is_high_phase = !is_high_phase;

        if (is_high_phase)
            current_rad_s = Generate_Random_Float(VSPIN_HIGH_MIN_RPM, VSPIN_HIGH_MAX_RPM) * RPM_TO_RAD_S;
        else
            current_rad_s = Generate_Random_Float(VSPIN_LOW_MIN_RPM, VSPIN_LOW_MAX_RPM) * RPM_TO_RAD_S;

        hold_timer    = now;
        hold_duration = (uint32_t)Generate_Random_Float(VSPIN_HOLD_MIN_MS, VSPIN_HOLD_MAX_MS);
    }

    /* 直接输出阶跃指令，让底盘板 PID + 机械惯性自然平滑 */
    return current_rad_s;
}

float X_speed, Y_speed, R_speed;
/*************************** Mode_Switch *****************************/

/**
 * @brief 模式切换
 * @author Nas(1319621819@qq.com)
 */
void Mode_Switch()
{
    float sin_beta, cos_beta;

    /* 1. 根据模式分配输入量与角度策略 */
    switch (Global.Chassis.mode)
    {
    case FLOW_Chassis:
    case FLOW_Gimbal:
    case SPIN_P:
    case SPIN_N:
        nav_offline_dash_active = 0;
        nav_offline_dash_finished = 0;

        /* 遥控器输入处理 */
        Global.Chassis.input.x = (RC_data.rc.ch[1] / 2.0f * WHEEL_RADIUS) / (30 * Rotation_radius);
        Global.Chassis.input.y = (-RC_data.rc.ch[0] / 2.0f * WHEEL_RADIUS) / (30 * Rotation_radius);

        /* 计算相对角度 */
        Chassis.relative_angle = Chassis_AngleLimit(relative_angle);
       

        /* 各模式下的旋转量处理 */
        if (Global.Chassis.mode == FLOW_Chassis)
        {
            Chassis.relative_angle = 0;
             Global.Chassis.input.r =  (-RC_data.rc.ch[2] / 5.0f * WHEEL_RADIUS) / (30 * Rotation_radius);  
        }
        else if (Global.Chassis.mode == FLOW_Gimbal)
        {
            Global.Chassis.input.r = 0;
        }
        else if (Global.Chassis.mode == SPIN_P)
        {
            Global.Chassis.input.r = Variable_Spin_Update();       
        }
        else // SPIN_N
        {
            Global.Chassis.input.r = -150 * RPM_TO_RAD_S;
        }
        break;

    case Navigation:

        Chassis.relative_angle = Chassis_AngleLimit(relative_angle);
    
/*         if(Navigation_online != 0 )
        { */
            if (Navigation_receive_1.x_speed != 0 || Navigation_receive_1.y_speed != 0)
        {
            Global.Chassis.input.x = Navigation_receive_1.x_speed;
            Global.Chassis.input.y = Navigation_receive_1.y_speed;
            Global.Chassis.input.r = 0;//60 * RPM_TO_RAD_S;
        }
        else
        {
            Global.Chassis.input.x = 0;
            Global.Chassis.input.y = 0;
                        if(Referee.game_progress == 4)
                            {
                                // 检测到掉血后锁存高速状态，直到退出比赛阶段
                                if (Referee.current_HP < Last_Hp)
                                {
                                    hurt_high_spin_latched = 1;
                                }

                                if (hurt_high_spin_latched)
                                {
                                    Global.Chassis.input.r = Variable_Spin_Update();
                                }
                                else
                                {
                                    Global.Chassis.input.r = 80 * RPM_TO_RAD_S;
                                }
                            }
                        else
                        {
                            hurt_high_spin_latched = 0;
                            Global.Chassis.input.r = 0.0f;
                        }
                        Last_Hp = Referee.current_HP;
        }
        break;
    }
        sin_beta = sinf(Chassis.relative_angle / 180.0f * PI);
        cos_beta = cosf(Chassis.relative_angle / 180.0f * PI);

        X_speed = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
        Y_speed = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
        R_speed = Global.Chassis.input.r;
}


/*-------------------- Task --------------------*/

/**
 * @brief          底盘任务
 * @param          none
 * @retval         none
 */
void Chassis_Tasks(void)
{
#if (USE_CHASSIS !=0 )
    if(Navigation_online > 0) Navigation_online--;
    //模式切换
    Mode_Switch();
    
#endif
}


/*-------------------- Set --------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0) // 符号相反
    RampGenerator_SetCurrent(&Vx_ramp, 0.0f);
/*      Global.Chassis.input.x = x; */
}


// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);
/*      Global.Chassis.input.y = y; */
}


// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}


// @brief 设置斜坡规划器加速度
 
void Chassis_SetAccel(float acc)
{

}

static void Pack_XY_Speed(uint8_t data[8])
{
    float_to_bytes(X_speed,                             &data[0]);
    float_to_bytes(Y_speed,                             &data[4]);
}

static void Pack_R_Yaw(uint8_t data[8])
{
    /* 原子拷贝 relative_angle，防止 CAN ISR 半写导致的 torn read */
    __disable_irq();
    float ra = relative_angle;
    __enable_irq();

    float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    
    if (AutoAim_ShouldBigYawFollow())
    {
        // 自瞄跟踪 或 扫描: 大 yaw 都跟随小 yaw，避免切扫描时大 yaw 目标突变
        Gimbal.big_yaw.big_yaw_location_set = ra + encoder_relative;
    }
    else
    {
        // 非自瞄（遥控/导航）
        Gimbal.big_yaw.big_yaw_location_set = Global.Gimbal.input.yaw; 
    }
    
    float_to_bytes(R_speed,                             &data[0]);
    float_to_bytes(Gimbal.big_yaw.big_yaw_location_set, &data[4]);
}

static void Pack_Control_Mode(uint8_t data[8])
{
    float_to_bytes(Global.Control.mode,                 &data[0]);
    float_to_bytes(Global.Chassis.mode,                 &data[4]);
}

static void Pack_IMU_Attitude(uint8_t data[8])
{
    float_to_bytes(imu_chassis.pitch,                   &data[0]);
    float_to_bytes(imu_chassis.yaw_cnt,                 &data[4]);
}

static void Pack_IMU_Gyro(uint8_t data[8])
{
    float_to_bytes(imu_chassis.gyro[0],                 &data[0]);
    float_to_bytes(imu_chassis.gyro[2],                 &data[4]);
}

static void Pack_Saltation_Mode(uint8_t data[8])
{
    float_to_bytes(Navigation_receive_1.yaw_speed,      &data[0]);
    uint8_to_bytes(Navigation_receive_1.running_state,  &data[4]); 
    uint8_to_bytes(fromMINIPC.mode,                     &data[5]);
    uint8_to_bytes(AA.is_scanning,                      &data[6]);
    uint8_to_bytes(Navigation_receive_1.region_code,    &data[7]);
}


static void Pack_TRIGGER_MODE(uint8_t data[8])
{
    float_to_bytes(Global.Shoot.trigger_mode,  &data[0]);    
    float_to_bytes(Global.Auto.mode,           &data[4]);
}



/*  分频调度表 —— IMU 源 1kHz(TIM13), 全速发送
 *                                       div  phase
 *  tick:  0   1   2   3   4   5   6   7   8   9
 *  帧数:  5   4   5   4   5   4   4   5   4   5   */
static const CanTxEntry_t ChassisTxTable[] = {
    /* 1kHz 速度+IMU: 控制环核心 */
    { CAN_ID_CHASSIS_SPEED_XY,     Pack_XY_Speed,       1, 0 },
    { CAN_ID_CHASSIS_SPEED_R_YAW,  Pack_R_Yaw,          1, 0 },
    { CAN_ID_CHASSIS_IMU_ATTITUDE, Pack_IMU_Attitude,   1, 0 },
    { CAN_ID_CHASSIS_IMU_GYRO,     Pack_IMU_Gyro,       1, 0 },
    /* 200Hz 模式/状态, 各自错开 */
    { CAN_ID_CHASSIS_MODE,         Pack_Control_Mode,   5, 0 },
    { CAN_ID_SHOOT_TRIGGER_MODE,   Pack_TRIGGER_MODE,   5, 2 },
    { CAN_ID_SALTATION_MODE,       Pack_Saltation_Mode, 5, 4 },
};

void Chassis_CAN_SendAll(void)
{
    static uint8_t tick = 0;
    static enum trigger_mode_e last_trigger_mode = TRIGGER_CLOSE;
    static enum control_mode_e last_control_mode = RC;
    static enum chassis_mode_e last_chassis_mode = FLOW_Chassis;
    static enum auto_mode_e    last_auto_mode    = NONE;
    static uint8_t trigger_mode_fast_resend = 0;
    static uint8_t mode_fast_resend = 0;
    uint8_t buf[8];

    if (Global.Shoot.trigger_mode != last_trigger_mode || Global.Auto.mode != last_auto_mode)
    {
        last_trigger_mode = Global.Shoot.trigger_mode;
        last_auto_mode    = Global.Auto.mode;
        trigger_mode_fast_resend = 5; /* 边沿后连续 5ms 以 1kHz 快速重发 */
    }

    if (Global.Control.mode != last_control_mode || Global.Chassis.mode != last_chassis_mode)
    {
        last_control_mode = Global.Control.mode;
        last_chassis_mode = Global.Chassis.mode;
        mode_fast_resend = 5; /* 模式边沿与 trigger 一样快速补发 */
    }

    for (uint8_t i = 0; i < sizeof(ChassisTxTable)/sizeof(ChassisTxTable[0]); i++) {
        uint8_t should_send = (tick % ChassisTxTable[i].divider == ChassisTxTable[i].phase);

        if (ChassisTxTable[i].id == CAN_ID_SHOOT_TRIGGER_MODE && trigger_mode_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_MODE && mode_fast_resend > 0) {
            should_send = 1;
        }

        if (should_send) {
            memset(buf, 0, sizeof(buf));
            ChassisTxTable[i].pack(buf);
            if (Fdcanx_SendData(&hfdcan2, ChassisTxTable[i].id, buf, 8) == 0) {
                if (ChassisTxTable[i].id == CAN_ID_SHOOT_TRIGGER_MODE && trigger_mode_fast_resend > 0) {
                    trigger_mode_fast_resend--;
                }
                if (ChassisTxTable[i].id == CAN_ID_CHASSIS_MODE && mode_fast_resend > 0) {
                    mode_fast_resend--;
                }
            }
        }
    }

    if (++tick >= 10) tick = 0;  /* LCM(1,2,5)=10, 10ms 一个完整周期 */
}
