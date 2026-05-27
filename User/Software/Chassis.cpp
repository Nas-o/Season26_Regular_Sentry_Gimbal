/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-22 15:52:02
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

#include "Auto_control.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"
#include "remote_control.hpp"
#include "Shoot.hpp"

#include "referee_system.hpp"
#include "supercup.hpp"
#include "Stm32_time.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"

#include "User_math.hpp"
#include "robot_param.hpp"
#include "ramp_generator.hpp"


Chassis_t Chassis ;
ChassisLinkMotion_t ChassisLinkMotion = {0};
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};   
RC_ctrl_t RC_data;
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
int Last_Hp = 400;
static uint32_t hurt_tick = 0;          // 最近一次掉血的时刻
static uint32_t hurt_high_duration_ms = 3000; // 掉血后保持高速的时长(ms)
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
    #define VSPIN_HIGH_MIN_RPM   85.0f    /* 高速段下限 RPM    */
    #define VSPIN_HIGH_MAX_RPM   95.0f    /* 高速段上限 RPM    */
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
            Global.Chassis.input.x = Navigation_receive_1.x_speed ;
            Global.Chassis.input.y = Navigation_receive_1.y_speed ;
            if(Navigation_receive_1.region_code == 11)
            {
                Global.Chassis.input.r = 70 * RPM_TO_RAD_S;
            }
            else
            {
                Global.Chassis.input.r = 0;
            }
            
        }
        else
        {
            Global.Chassis.input.x = 0;
            Global.Chassis.input.y = 0;
                        if(Referee.game_progress == 4)
                        {
                            // 检测掉血，刷新计时
                            if (Referee.current_HP < Last_Hp)
                                hurt_tick = HAL_GetTick();

                            // 掉血后一段时间内高速，否则低速
                            if ((HAL_GetTick() - hurt_tick) < hurt_high_duration_ms)
                                Global.Chassis.input.r = -Variable_Spin_Update();
                            else
                                Global.Chassis.input.r = -50 * RPM_TO_RAD_S;
                        }
                        else
                        {
                            Global.Chassis.input.r = 0.0f;
                        }
                        Last_Hp = Referee.current_HP;
        }
        break;
    }
        sin_beta = sinf(Chassis.relative_angle / 180.0f * PI);
        cos_beta = cosf(Chassis.relative_angle / 180.0f * PI);

        ChassisLinkMotion.x = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
        ChassisLinkMotion.y = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
        ChassisLinkMotion.r = Global.Chassis.input.r;
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

