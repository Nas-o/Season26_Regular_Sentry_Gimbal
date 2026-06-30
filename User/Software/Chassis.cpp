/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-05 20:00:52
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Chassis.cpp
 * @brief  底盘控制器 
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


ChassisController Chassis;                          // 底盘控制器唯一实例
robot::chassis::ChassisLinkMotion ChassisLinkMotion; // 底盘连杆运动解算结果 (机体系→世界系旋转后)
RcCtrl RC_data;                                     // 统一遥控数据 (从DT7/VT13/FSI6X汇聚)
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;           // X/Y/旋转 三轴斜坡发生器 (平滑加减速)
int Last_Hp = 400;                                  // 上一拍血量, 用于受击检测

namespace 
{
    uint8_t nav_offline_dash_active = 0;                // 导航离线冲刺激活标志
    uint8_t nav_offline_dash_finished = 0;              // 导航离线冲刺已完成标志
    uint32_t hurt_tick = 0;                             // 最后一次掉血时刻 (ms)
    uint32_t hurt_high_duration_ms = 3000;              // 受击后高速小陀螺持续时间 (ms)
    float nav_offline_dash_start_s = 0.0f;              // 导航离线冲刺开始时间 (s)
} // namespace

/*-------------------- Public: init --------------------*/

/**
 * @brief  底盘控制器初始化
 * @note   在RTOS启动前调用一次, 设置默认模式和随机数种子
 */
void ChassisController::init()
{
    Global.Chassis.mode = FLOW_Chassis;  // 默认: 底盘跟随模式
    random.interval = 250;               // 随机变速陀螺换向间隔 (ms)
    srand(HAL_GetTick());                // 用系统时钟初始化随机种子
}

/*-------------------- Static: angleLimit --------------------*/

/**
 * @brief  将角度限制在 [-180, 180] 范围内
 * @param  angle 输入角度 (度)
 * @return 归一化后的角度 (度)
 * @note   通过计算超出180°的整数倍来修正, 避免使用fmod的浮点精度问题
 */
float ChassisController::angleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;  // 超过180°的倍数
    if (angle > 180.0f)
    {
        if (mul % 2 == 1)
            angle -= (mul + 1) * 180.0f;
        else
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1)
            angle += (mul + 1) * 180.0f;
        else
            angle += mul * 180.0f;
    }
    return angle;
}

/*-------------------- Static: obtainModulusNormalization --------------------*/

/**
 * @brief  模值归一化: 将数值映射到 [-modulus/2, modulus/2) 区间
 * @param  x        待归一化的值
 * @param  modulus   模值 (如360°对应modulus=360)
 * @return 归一化后的值, 属于 [-modulus/2, modulus/2)
 * @note   常用于角度差值的最短路径计算
 */
float ChassisController::obtainModulusNormalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}

/*-------------------- Public: randomSpin --------------------*/

/**
 * @brief  随机小陀螺速度生成 (固定间隔换一次随机转速)
 * @param  min  最小随机转速
 * @param  max  最大随机转速
 * @return 当前周期的小陀螺转速
 * @note   每隔 random.interval 次调用重新生成一次随机值;
 *         连续两次低速后强制输出高速, 避免长时间低速被瞄准
 */
float ChassisController::randomSpin(float min, float max)
{
    static int s_run_count = 0;
	s_run_count++;
	float target_r;
	if (s_run_count >= random.interval)  // 到达换速间隔
	{
		if (random.low_count < 2)       // 低速次数未超限
		{
			random.value = generateRandomFloat(min, max);
			if (random.value < 7.0f)    // 本次为低速
			{
				random.low_count++;
			}
		}
		else  // 连续两次低速 → 强制高速
		{
			random.value = 9.0f;
			random.low_count = 0;
		}
		s_run_count = 0;
	 }
	 return random.value;
}

/*-------------------- Static: generateRandomFloat --------------------*/

/**
 * @brief  生成 [min, max] 范围内的随机浮点数
 * @param  min  下界
 * @param  max  上界
 * @return 均匀分布的随机浮点值
 */
float ChassisController::generateRandomFloat(float min, float max)
{
	return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

/*-------------------- Public: variableSpinUpdate --------------------*/

/**
 * @brief  变速小陀螺更新 —— 高低两档随机切换
 * @return 当前小陀螺角速度 (rad/s)
 * @note   在高速档(85~95 RPM)和低速档(60~70 RPM)之间随机切换,
 *         每次保持 300~800ms 后换档, 增加对方预测难度
 */
float ChassisController::variableSpinUpdate()
{
    #define VSPIN_LOW_MIN_RPM    60.0f
    #define VSPIN_LOW_MAX_RPM    70.0f
    #define VSPIN_HIGH_MIN_RPM   85.0f
    #define VSPIN_HIGH_MAX_RPM   95.0f
    #define VSPIN_HOLD_MIN_MS    300
    #define VSPIN_HOLD_MAX_MS    800

    static float    s_current_rad_s = 0.0f;
    static uint32_t s_hold_timer    = 0;
    static uint32_t s_hold_duration = 0;
    static uint8_t  s_is_high_phase = 0;
    static uint8_t  s_initialized   = 0;

    uint32_t now = HAL_GetTick();

    if (!s_initialized)
    {
        s_current_rad_s = generateRandomFloat(VSPIN_LOW_MIN_RPM, VSPIN_LOW_MAX_RPM) * RPM_TO_RAD_S;
        s_hold_timer    = now;
        s_hold_duration = (uint32_t)generateRandomFloat(VSPIN_HOLD_MIN_MS, VSPIN_HOLD_MAX_MS);
        s_is_high_phase = 0;
        s_initialized   = 1;
    }

    if (now - s_hold_timer >= s_hold_duration)
    {
        s_is_high_phase = !s_is_high_phase;

        if (s_is_high_phase)
            s_current_rad_s = generateRandomFloat(VSPIN_HIGH_MIN_RPM, VSPIN_HIGH_MAX_RPM) * RPM_TO_RAD_S;
        else
            s_current_rad_s = generateRandomFloat(VSPIN_LOW_MIN_RPM, VSPIN_LOW_MAX_RPM) * RPM_TO_RAD_S;

        s_hold_timer    = now;
        s_hold_duration = (uint32_t)generateRandomFloat(VSPIN_HOLD_MIN_MS, VSPIN_HOLD_MAX_MS);
    }

    return s_current_rad_s;
}

/*-------------------- Public: modeSwitch --------------------*/

/**
 * @brief  底盘模式切换与速度解算
 * @note   根据 Global.Chassis.mode 选择不同的底盘运动策略:
 *         - FLOW_Chassis: 底盘单独控制
 *         - FLOW_Gimbal:  云台跟随模式
 *         - SPIN_P:       正向变速小陀螺
 *         - SPIN_N:       反向固定速小陀螺
 *         - Navigation:   导航模式
 *
 *         最后将机体系速度通过小正角旋转矩阵变换为世界系速度
 */
void ChassisController::modeSwitch()
{
    float sin_beta, cos_beta;

    switch (Global.Chassis.mode)
    {
    case FLOW_Chassis:
    case FLOW_Gimbal:
    case SPIN_P:
    case SPIN_N:
        nav_offline_dash_active = 0;
        nav_offline_dash_finished = 0;

        // 遥控器摇杆 → 底盘X/Y速度 (rpm→rad/s 线速换算)
        Global.Chassis.input.x = (RC_data.rc.ch[1] / 2.0f * WHEEL_RADIUS) / (30 * Rotation_radius);
        Global.Chassis.input.y = (-RC_data.rc.ch[0] / 2.0f * WHEEL_RADIUS) / (30 * Rotation_radius);

        // 编码器相对角度归一化到 [-180, 180]
        relative_angle = angleLimit(::relative_angle);

        if (Global.Chassis.mode == FLOW_Chassis)
        {
            relative_angle = 0; 
             Global.Chassis.input.r =  (-RC_data.rc.ch[2] / 5.0f * WHEEL_RADIUS) / (30 * Rotation_radius);  // 手动旋转
        }
        else if (Global.Chassis.mode == FLOW_Gimbal)
        {
            Global.Chassis.input.r = 0;
        }
        else if (Global.Chassis.mode == SPIN_P)
        {
            Global.Chassis.input.r = variableSpinUpdate();  // 正向变速小陀螺
        }
        else // SPIN_N
        {
            Global.Chassis.input.r = -150 * RPM_TO_RAD_S;   // 反向固定速小陀螺
        }
        break;

    case Navigation:

        relative_angle = angleLimit(::relative_angle);
    
        // 导航有速度指令时: 直接使用ROS下发的平移速度
            if (Navigation_receive_1.x_speed != 0 || Navigation_receive_1.y_speed != 0)
        {
            Global.Chassis.input.x = Navigation_receive_1.x_speed ;
            Global.Chassis.input.y = Navigation_receive_1.y_speed ;
            // 区域11 = 小陀螺区域, 导航期间执行小陀螺
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
            // 导航无速度指令: 原地小陀螺
            Global.Chassis.input.x = 0;
            Global.Chassis.input.y = 0;
                        if(Referee.game_progress == 4)  // 比赛进行中
                        {
                            // 受击检测: 血量下降 → 记录时刻
                            if (Referee.current_HP < Last_Hp)
                                hurt_tick = HAL_GetTick();

                            // 受击后3s内高速小陀螺, 否则低速小陀螺
                            if ((HAL_GetTick() - hurt_tick) < hurt_high_duration_ms)
                                Global.Chassis.input.r = -variableSpinUpdate();
                            else
                                Global.Chassis.input.r = -50 * RPM_TO_RAD_S;
                        }
                        else
                        {
                            Global.Chassis.input.r = 0.0f;  // 未开赛: 不旋转
                        }
                        Last_Hp = Referee.current_HP;
        }
        break;
    }
        // 坐标系变换: 机体系 → 世界系 (relative_angle为大yaw编码器角度)
        sin_beta = sinf(relative_angle / 180.0f * PI);
        cos_beta = cosf(relative_angle / 180.0f * PI);

        // 旋转矩阵变换: R(-β) * [vx, vy]^T
        ::ChassisLinkMotion.x = -(Global.Chassis.input.x * cos_beta - sin_beta * Global.Chassis.input.y);
        ::ChassisLinkMotion.y = Global.Chassis.input.x * sin_beta + Global.Chassis.input.y * cos_beta;
        ::ChassisLinkMotion.r = Global.Chassis.input.r;
}

/*-------------------- Public: task --------------------*/

/**
 * @brief  底盘控制主任务入口 (1kHz)
 * @note   由FreeRTOS的Chassis_Task调用, 每1ms执行一次
 *         USE_CHASSIS=0 时整个底盘控制被编译裁剪
 */
void ChassisController::task()
{
#if (USE_CHASSIS !=0 )
    if(Navigation_online > 0) Navigation_online--;  // 导航在线计时器递减
    modeSwitch();                                    // 模式切换+速度解算
#endif
}

/*-------------------- Public: setX --------------------*/

/**
 * @brief  设置X轴目标速度 (带斜坡平滑)
 * @param  x 目标速度
 * @note   当方向反转时立即将斜坡当前值置零, 实现快速换向
 */
void ChassisController::setX(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0)  // 方向反转
    RampGenerator_SetCurrent(&Vx_ramp, 0.0f);         // 立即清零
}

/*-------------------- Public: setY --------------------*/

/**
 * @brief  设置Y轴目标速度 (带斜坡平滑)
 * @param  y 目标速度
 */
void ChassisController::setY(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0)  // 方向反转
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);    // 立即清零
}

/*-------------------- Public: setR --------------------*/

/**
 * @brief  设置旋转速度
 * @param  r 目标旋转角速度 (rad/s)
 */
void ChassisController::setR(float r)
{
    Global.Chassis.input.r = r;
}

/*-------------------- Public: setAccel --------------------*/

/**
 * @brief  设置加速度 (预留接口, 暂未实现)
 * @param  acc 加速度值
 */
void ChassisController::setAccel(float acc)
{
}


/* ============================================================
 *  C兼容包装函数 (供FreeRTOS任务 / C源文件调用)
 *  内部转发给 ChassisController 类实例
 * ============================================================ */

void Chassis_Init()         { Chassis.init(); }
void Chassis_Tasks()        { Chassis.task(); }
void Chassis_SetX(float x)  { Chassis.setX(x); }
void Chassis_SetY(float y)  { Chassis.setY(y); }
void Chassis_SetR(float r)  { Chassis.setR(r); }
void Chassis_SetAccel(float acc) { Chassis.setAccel(acc); }
void Mode_Switch()          { Chassis.modeSwitch(); }
float Random_Spin(float min, float max) { return Chassis.randomSpin(min, max); }
float Generate_Random_Float(float min, float max) { return ChassisController::generateRandomFloat(min, max); }
float Variable_Spin_Update(void) { return Chassis.variableSpinUpdate(); }

