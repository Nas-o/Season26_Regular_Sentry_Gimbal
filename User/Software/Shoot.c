/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       shoot.c/h
  * @brief      发射机构控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建发射机构代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#include "Shoot.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "referee_system.h"
#include "User_math.h"

Shoot_t Shoot;





/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Shoot_Init()
{
	// 摩擦轮电机初始化
	SHOOTMotor_init(DJI_M3508, ShootMotor_L);
	SHOOTMotor_init(DJI_M3508, ShootMotor_R);


	// 摩擦轮电机
	PID_Set(&Shoot.shoot_L_speed_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
	PID_Set(&Shoot.shoot_R_speed_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);

}


/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Shoot_Updater()
{
	/*------状态量更新------*/
	//速度
	Shoot.shoot_speed_L_now = (SHOOTMotor_get_data(ShootMotor_L).speed_rpm);
	Shoot.shoot_speed_R_now = (SHOOTMotor_get_data(ShootMotor_R).speed_rpm);

	/*------目标量更新------*/
    //摩擦轮
	static enum shoot_mode_e pre_shoot_mode = CLOSE;

// 模式切换时的启动逻辑
if (pre_shoot_mode == CLOSE && Global.Shoot.shoot_mode != CLOSE) {
    // 从关闭状态启动
    Shoot.shoot_speed_set = SHOOT_SPEED_BEGIN;
    Global.Shoot.shoot_status = NOK;
    pre_shoot_mode = Global.Shoot.shoot_mode;
} 
else {
    // 正常工作状态
    switch(Global.Shoot.shoot_mode) {
        case CLOSE:
            Shoot.shoot_speed_set = SHOOT_SPEED_CLOSE;
            Global.Shoot.shoot_status = NOK;
            break;
        case READY:
            Shoot.shoot_speed_set = SHOOT_SPEED_READY;
            {
                const float shoot_speed_l_target = (float)SHOOT_SPEED_READY;
                const float shoot_speed_r_target = -(float)SHOOT_SPEED_READY;

                if ((fabsf(Shoot.shoot_speed_L_now - shoot_speed_l_target) < 300.0f) &&
                    (fabsf(Shoot.shoot_speed_R_now - shoot_speed_r_target) < 300.0f)) {
                    Global.Shoot.shoot_status = OK;
                } else {
                    Global.Shoot.shoot_status = NOK;
                }
            }
            break;
        case DEBUG_SHOOT:
            Shoot.shoot_speed_set = SHOOT_SPEED_BEGIN;
            Global.Shoot.shoot_status = OK;
            break;
        default:
            Shoot.shoot_speed_set = SHOOT_SPEED_CLOSE;
            Global.Shoot.shoot_status = NOK;
            break;
    }
    pre_shoot_mode = Global.Shoot.shoot_mode;
}


}


/*-------------------- Trigger Arbitrate --------------------*/

/**
 * @brief  全工程唯一写 Global.Shoot.trigger_mode 的地方
 *         两档直通: 自瞄走 mcu_fire_permit, 手动走 manual_trigger_mode
 */
static void Shoot_SelectTriggerMode(void)
{
    if (Global.Control.mode == LOCK || Global.Shoot.shoot_mode == CLOSE)
    {
        Global.Shoot.trigger_mode = TRIGGER_CLOSE;
    }
    else if (Global.Auto.mode != NONE)
    {
        Global.Shoot.trigger_mode = Global.Shoot.mcu_fire_permit ? ONLY_AIM_HIGH : TRIGGER_CLOSE;
    }
    else
    {
        Global.Shoot.trigger_mode = (Global.Shoot.shoot_status == OK) ? Global.Shoot.manual_trigger_mode : TRIGGER_CLOSE;
    }
}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Shoot_Calculater()
{
	Shoot.current[0] = PID_Cal(&Shoot.shoot_L_speed_pid, Shoot.shoot_speed_L_now, Shoot.shoot_speed_set);
	Shoot.current[1] = PID_Cal(&Shoot.shoot_R_speed_pid, Shoot.shoot_speed_R_now, -Shoot.shoot_speed_set);  
}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Shoot_Controller()
{
	SHOOTMotor_set(Shoot.current[0],ShootMotor_L);
	SHOOTMotor_set(Shoot.current[1],ShootMotor_R);

}


/*-------------------- Task --------------------*/

/**
 * @brief          发射机构任务
 * @param          none
 * @retval         none
 */
void Shoot_Tasks(void)
{
#if (USE_SHOOT != 0)
	// 发射机构数据更新 (摩擦轮达速检测)
	Shoot_Updater();
	// 开火仲裁 (唯一写 trigger_mode 的地方)
	Shoot_SelectTriggerMode();
	// 发射机构控制计算
	Shoot_Calculater();
	// 发射机构电机控制
	Shoot_Controller();

#endif
}

