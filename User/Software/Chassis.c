/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-04-07 17:33:29
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
    PID_Set(&Chassis.chassis_follow_pid, 0.65f, 0.0f, 0.0f, 0.0f, 120, 40);
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
 *         1) 转速分为 LOW [50,80] RPM 和 HIGH [120,170] RPM
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
    #define VSPIN_LOW_MIN_RPM    55.0f     /* 低速段下限 RPM    */
    #define VSPIN_LOW_MAX_RPM    70.0f     /* 低速段上限 RPM    */
    #define VSPIN_HIGH_MIN_RPM   90.0f    /* 高速段下限 RPM    */
    #define VSPIN_HIGH_MAX_RPM   110.0f    /* 高速段上限 RPM    */
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
    /* ---- 跟随连续角状态 ---- */
    static float last_raw_angle = 0.0f;       // 上一拍归一化角度
    static float relative_angle_cnt = 0.0f;   // 解缠后的连续角
    static uint8_t prev_mode_follow = FLOW_Chassis;
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
        Chassis.relative_angle = relative_angle;
        Chassis.relative_angle = Chassis_AngleLimit(Chassis.relative_angle);

        /* 各模式下的旋转量处理 */
        if (Global.Chassis.mode == FLOW_Chassis)
        {
            Chassis.relative_angle = 0;
            /* if (Is_Relative_Angle_Online())
            { */
/*             if (fabs(Chassis.relative_angle) > 2.0f)
            {
                // 出现180度振荡说明是正反馈，PID输出需要反向
                Global.Chassis.input.r = -PID_Cal(&Chassis.chassis_follow_pid, Chassis.relative_angle, 0) * RPM_TO_DEG_S; 
            }
            else
            {
                Global.Chassis.input.r = 0;
            } */
            /* }
            else
            {
                // 通信中断时，停止自旋跟随，防止疯转
                R_speed = 0;
            } */
             Global.Chassis.input.r =  (-RC_data.rc.ch[2] / 5.0f * WHEEL_RADIUS) / (30 * Rotation_radius);  
        }
        else if (Global.Chassis.mode == FLOW_Gimbal)
        {
            if (fabs(Chassis.relative_angle) > 5.0f)
            {
                // 出现180度振荡说明是正反馈，PID输出需要反向
                Global.Chassis.input.r = -PID_Cal(&Chassis.chassis_follow_pid, Chassis.relative_angle, 0) * RPM_TO_RAD_S; 
            }
            else
            {
                Global.Chassis.input.r = 0;
            }
        }
        // else if (Global.Chassis.mode == FLOW_Gimbal)
        // {
        //     /* ---- 切入瞬间：重置状态 ---- */
        //     if (prev_mode_follow != FLOW_Gimbal)
        //     {
        //         last_raw_angle      = Chassis.relative_angle;
        //         relative_angle_cnt  = Chassis.relative_angle;
        //     }

        //     /* ---- 角度解缠：增量 + 归一化 ---- */
        //     float delta = Chassis.relative_angle - last_raw_angle;
        //     delta = Obtain_Modulus_Normalization(delta, 360.0f);   // → [-180, 180]
        //     relative_angle_cnt += delta;
        //     last_raw_angle = Chassis.relative_angle;

        //     /* ---- 外环P + 陀螺仪阻尼 ---- */
        //     static float kp_follow = 1.0f;
        //     static float kd_gyro   = 0.5f;
        //     static float w_max     = 12.0f;

        //     if (fabs(relative_angle_cnt) > 3.0f)
        //     {
        //         // 外环：角度误差 → 目标角速度 (rad/s)
        //         float target_w = kp_follow * relative_angle_cnt * RPM_TO_RAD_S;

        //         // 限幅目标角速度
        //         if (target_w >  w_max) target_w =  w_max;
        //         if (target_w < -w_max) target_w = -w_max;

        //         // 阻尼：直接用底盘陀螺仪 yaw 角速度 (rad/s)
        //         // ★ 注意符号：需确认 input.r > 0 时 gyro[2] 的正负
        //         //   如果同号 → 用减号（阻尼对抗运动）
        //         //   如果反号 → 用加号
        //         Global.Chassis.input.r = target_w - kd_gyro * imu_chassis.gyro[2];

        //         // 总输出限幅
        //         if (Global.Chassis.input.r >  w_max) Global.Chassis.input.r =  w_max;
        //         if (Global.Chassis.input.r < -w_max) Global.Chassis.input.r = -w_max;
        //     }
        //     else
        //     {
        //         // 死区内：不转，同步连续角防漂移
        //         Global.Chassis.input.r = 0;
        //         relative_angle_cnt = Chassis.relative_angle;
        //         last_raw_angle     = Chassis.relative_angle;
        //     }
        // }
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

        Chassis.relative_angle = relative_angle;
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
                                    Global.Chassis.input.r = 108 * RPM_TO_RAD_S;
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
        
        
/*         if (Navigation_online != 0)
        {
            nav_offline_dash_active = 0;
            nav_offline_dash_finished = 0;

            Global.Chassis.input.x = Navigation_receive_1.x_speed * 200.0f;
            Global.Chassis.input.y = Navigation_receive_1.y_speed * 200.0f;
            Global.Chassis.input.r = 0;
        }
        else
        {
            uint8_t in_battle = (Referee_data.game_progress == PROGRESS_BATTLE) ||
                                (Referee.game_progress == PROGRESS_BATTLE);

            if (in_battle)
            {
                if (!nav_offline_dash_active && !nav_offline_dash_finished)
                {
                    nav_offline_dash_active = 1;
                    nav_offline_dash_start_s = Get_SysTime_s();
                }

                if (nav_offline_dash_active && (Get_SysTime_s() - nav_offline_dash_start_s < 3.0f))
                {
                    Chassis.relative_angle = 0.0f;
                    Global.Chassis.input.x = 200.0f;
                    Global.Chassis.input.y = 0.0f;
                    Global.Chassis.input.r = 0.0f;
                }
                else
                {
                    nav_offline_dash_active = 0;
                    nav_offline_dash_finished = 1;
                    Global.Chassis.input.x = 0.0f;
                    Global.Chassis.input.y = 0.0f;
                    Global.Chassis.input.r = 0.0f;
                }
            }
            else
            {
                nav_offline_dash_active = 0;
                nav_offline_dash_finished = 0;
                Global.Chassis.input.x = 0.0f;
                Global.Chassis.input.y = 0.0f;
                Global.Chassis.input.r = 0.0f;
            } */

           /*   if(Referee_data.game_progress == 4)
                            {
                                if((Referee_data.remain_hp >= Last_Hp && Get_SysTime_s() - Hp_Time_Wait > 5) || cap.remain_vol < 16)//没受到攻击就转的慢点
                                {
                                    R_speed = 60 * RPM_TO_DEG_S;
                                }
                                else//检测到掉血就高速或者变速
                                {
                                    if(R_speed == 60 * RPM_TO_DEG_S)//Random_Spin(7.5,9.5))
                                    Hp_Time_Wait = Get_SysTime_s();
                                    R_speed = Random_Spin(7.5,9.5);

                                }
                            }
                        else  
            
             Chassis.speed.now_r = R_speed; 
                        Last_Hp = Referee_data.remain_hp;
                        Navigation_receive_1.header = 0;
                        Navigation_receive_1.checksum = 0;
                        Navigation_receive_1.x_speed  = 0;
                        Navigation_receive_1.y_speed = 0;
                        Navigation_receive_1.yaw_speed = 0;
                        Navigation_receive_1.rotate = 0;
                        Navigation_receive_1.running_state = 0;  */
/*         }
        else
        {
            Global.Chassis.input.x = 0.0f;
            Global.Chassis.input.y = 0.0f;
            Global.Chassis.input.r = 0.0f;
        } */
        break;
    }
    prev_mode_follow = Global.Chassis.mode;
    /* 2. 统一运动解算 (Vector Decomposition) */
    sin_beta = sinf(Chassis.relative_angle / 180.0f * PI);
    cos_beta = cosf(Chassis.relative_angle / 180.0f * PI);
/*     if(Global.Chassis.mode == SPIN_N || Global.Chassis.mode == SPIN_P)
    {
        X_speed = -(Global.Chassis.input.x * cos_beta + sin_beta * Global.Chassis.input.y);
        Y_speed = -Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    }
    else
    {
        X_speed = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
        Y_speed = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
    } */
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
    float encoder_relative = (float)DJIMotor_GetData(SMALLYAWMotor).angle;
    
    if (AutoAim_ShouldBigYawFollow())
    {
        // 自瞄跟踪 或 扫描: 大 yaw 都跟随小 yaw，避免切扫描时大 yaw 目标突变
        Gimbal.big_yaw.big_yaw_location_set = relative_angle + encoder_relative;
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
    uint8_to_bytes(AA.is_scanning,                     &data[6]);
}


static void Pack_TRIGGER_MODE(uint8_t data[8])
{
    float_to_bytes(Global.Shoot.tigger_mode,  &data[0]);    
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
    uint8_t buf[8];

    for (uint8_t i = 0; i < sizeof(ChassisTxTable)/sizeof(ChassisTxTable[0]); i++) {
        if (tick % ChassisTxTable[i].divider == ChassisTxTable[i].phase) {
            ChassisTxTable[i].pack(buf);
            Fdcanx_SendData(&hfdcan2, ChassisTxTable[i].id, buf, 8);
        }
    }

    if (++tick >= 10) tick = 0;  /* LCM(1,2,5)=10, 10ms 一个完整周期 */
}