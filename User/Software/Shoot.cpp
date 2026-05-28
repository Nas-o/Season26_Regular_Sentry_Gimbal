/**
 * @file  Shoot.cpp
 * @brief 发射机构控制器 —— 摩擦轮+拨弹+射速自适应
 */
#include "Shoot.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "referee_system.hpp"
#include "User_math.hpp"
#include "Auto_control.hpp"
#include "shoot_speed_adapt.hpp"

Shoot_t Shoot;

/*-------------------- Init --------------------*/

void Shoot_Init()
{
	SHOOTMotor_init(DJI_M3508, ShootMotor_L);
	SHOOTMotor_init(DJI_M3508, ShootMotor_R);

	PID_Set(&Shoot.fric_L_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
	PID_Set(&Shoot.fric_R_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);

	ShootSpeedAdapt_Init();
}

/*-------------------- Update --------------------*/

void Shoot_Updater()
{
	Shoot.fric_spd_L = SHOOTMotor_get_data(ShootMotor_L).speed_rpm;
	Shoot.fric_spd_R = SHOOTMotor_get_data(ShootMotor_R).speed_rpm;

	static enum shoot_mode_e pre_mode = CLOSE;

	if (pre_mode == CLOSE && Global.Shoot.shoot_mode != CLOSE) {
		Shoot.fric_spd_set = SHOOT_SPEED_BEGIN;
		Global.Shoot.shoot_status = NOK;
		pre_mode = Global.Shoot.shoot_mode;
	} 
	else {
		switch(Global.Shoot.shoot_mode) {
			case CLOSE:
				Shoot.fric_spd_set = SHOOT_SPEED_CLOSE;
				Global.Shoot.shoot_status = NOK;
				break;
			case READY:
				Shoot.fric_spd_set = SHOOT_SPEED_READY + ShootSpeedAdapt_GetOffset();
				{
					const float target_l = (float)SHOOT_SPEED_READY;
					const float target_r = -(float)SHOOT_SPEED_READY;
					if ((fabsf(Shoot.fric_spd_L - target_l) < 300.0f) &&
						(fabsf(Shoot.fric_spd_R - target_r) < 300.0f))
						Global.Shoot.shoot_status = OK;
					else
						Global.Shoot.shoot_status = NOK;
				}
				break;
			case DEBUG_SHOOT:
				Shoot.fric_spd_set = SHOOT_SPEED_BEGIN;
				Global.Shoot.shoot_status = OK;
				break;
			default:
				Shoot.fric_spd_set = SHOOT_SPEED_CLOSE;
				Global.Shoot.shoot_status = NOK;
				break;
		}
		pre_mode = Global.Shoot.shoot_mode;
	}
}

/*-------------------- Trigger Arbitrate --------------------*/

static void Shoot_SelectTriggerMode(void)
{
    if (Global.Control.mode == LOCK || Global.Shoot.shoot_mode == CLOSE)
    {
        Global.Shoot.trigger_mode = TRIGGER_CLOSE;
    }
    else if (Global.Auto.mode != NONE)
    {
        if (!Global.Shoot.mcu_fire_permit)
            Global.Shoot.trigger_mode = TRIGGER_CLOSE;
        else if (toMINIPC.mode == MODE_SMALL_BUFF || toMINIPC.mode == MODE_BIG_BUFF)
            Global.Shoot.trigger_mode = SINGLE;
        else
            Global.Shoot.trigger_mode = ONLY_AIM_HIGH;
    }
    else
    {
        Global.Shoot.trigger_mode = (Global.Shoot.shoot_status == OK)
            ? Global.Shoot.manual_trigger_mode : TRIGGER_CLOSE;
    }
}

/*-------------------- Calculate --------------------*/

void Shoot_Calculater()
{
	Shoot.current[0] = PID_Cal(&Shoot.fric_L_pid, Shoot.fric_spd_L,  Shoot.fric_spd_set);
	Shoot.current[1] = PID_Cal(&Shoot.fric_R_pid, Shoot.fric_spd_R, -Shoot.fric_spd_set);
}

/*-------------------- Control --------------------*/

void Shoot_Controller()
{
	SHOOTMotor_set(Shoot.current[0], ShootMotor_L);
	SHOOTMotor_set(Shoot.current[1], ShootMotor_R);
}

/*-------------------- Task --------------------*/

void Shoot_Tasks(void)
{
#if (USE_SHOOT != 0)
	Shoot_Updater();
	Shoot_SelectTriggerMode();
	Shoot_Calculater();
	Shoot_Controller();
#endif
}

