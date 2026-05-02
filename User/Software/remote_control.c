#include "remote_control.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "ramp_generator.h"
#include "referee_system.h"
#include "Auto_control.h"

#include "DT7.h"
#include "VT13.h"
#include "FSI6X.h"
#include "Power_switch.h"
#include "IMU_updata.h"

#include "Stm32_time.h"
#include "Power_switch.h"

#include "cmsis_os2.h"

#define SHOOT_MODE_DEBOUNCE_TICKS 10


/*----------------------------------- 遥控数据更新 --------------------------------------*/
/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    RC_data.mouse.press_mid = 0;
    DT7_data.online--;
    RC_data.online = DT7_data.online;
}

/**
 * @brief 来自图传的遥控数据
 *
 */
void VT13toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // 挡位与拨杆映射
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
    {
        RC_data.rc.s[0] = RC_SW_UP;
        RC_data.rc.s[1] = RC_SW_UP;
    }
    // 滚轮与拨杆映射
    if (VT13_data.rc.wheel < -330)
        RC_data.rc.s[1] = RC_SW_DOWN;
    if ((VT13_data.rc.wheel > -330) && (VT13_data.rc.wheel < 330) && (VT13_data.rc.mode_sw != 2))
        RC_data.rc.s[1] = RC_SW_MID;
    if (VT13_data.rc.wheel >= 330)
        RC_data.rc.s[1] = RC_SW_UP;
    if (VT13_data.rc.left_button == 1)
        Power_TurnOff(power2);
    else
        Power_TurnOn(power2);
    /* if (VT13_data.rc.right_button == 1)
        GIMBALMotor_setzero(YAW_ZERO + 135.0f, YAWMotor);
    else
        GIMBALMotor_setzero(YAW_ZERO, YAWMotor); */

    /*键鼠数据 */
    RC_data.key.v = VT13_data.key.v;
    RC_data.mouse.x = VT13_data.mouse.x;
    RC_data.mouse.y = VT13_data.mouse.y;
    RC_data.mouse.z = VT13_data.mouse.z;
    RC_data.mouse.press_l = VT13_data.mouse.press_l;
    RC_data.mouse.press_r = VT13_data.mouse.press_r;
    RC_data.mouse.press_mid = VT13_data.mouse.middle;
    VT13_data.online--;
    RC_data.online = VT13_data.online;
}

/**
 * @brief 遥控数据来源于FS_I6X遥控器
 *
 */
void FSI6XtoRCdata()
{ 
   RC_data.rc.ch[0]=FSI6X_data.CH1;
   RC_data.rc.ch[1]=FSI6X_data.CH2;
   RC_data.rc.ch[2]=FSI6X_data.CH4;
   RC_data.rc.ch[3]=FSI6X_data.CH3;

   for(int i = 0 ; i<4 ; i++) // 死区判断
   {
    if(fabs(RC_data.rc.ch[i]) < 10)
    RC_data.rc.ch[i] = 0;
   }
   
   if(FSI6X_data.CH8==FS_DOWN)//LOCK
   {
    RC_data.rc.s[0]=RC_SW_DOWN;
    RC_data.rc.s[1]=RC_SW_DOWN;
   }
   else if (FSI6X_data.CH8==FS_UP)
   {
     if (FSI6X_data.CH7==FS_MID && FSI6X_data.CH6 == FS_UP)// 正小陀螺
     {
        RC_data.rc.s[0]=RC_SW_UP ;
        RC_data.rc.s[1]=RC_SW_MID;
     }
     else if (FSI6X_data.CH7==FS_DOWN && FSI6X_data.CH6 == FS_UP)// 逆小陀螺
     {
        RC_data.rc.s[1]=RC_SW_DOWN;
        RC_data.rc.s[0]=RC_SW_MID;
     }
     else if(FSI6X_data.CH6 == FS_DOWN && FSI6X_data.CH7 == FS_UP)// 开启摩擦轮
     {
       RC_data.rc.s[1]=RC_SW_UP; 
       RC_data.rc.s[0]=RC_SW_MID;
       
     }
     else
     {
       RC_data.rc.s[1]=RC_SW_MID; 
       RC_data.rc.s[0]=RC_SW_MID; 
     }
     if(FSI6X_data.CH5 == FS_DOWN) // 开火
       {
        RC_data.rc.ch[4] = 660;
       }
       else if(FSI6X_data.CH5 == FS_UP)
       {
        RC_data.rc.ch[4] = 0;
       }
   }
   FSI6X_data.online--;
   RC_data.online = FSI6X_data.online;
}

/**
 * @brief 根据数据来源更新遥控数据
 *
 */
void RCdata_Updater()
{
    if (DT7_data.online >= 0)
      DT7toRCdata();
    else if (FSI6X_data.online >= 0)
      FSI6XtoRCdata();
    else if (VT13_data.online >= 0)
      VT13toRCdata(); 
}


/*----------------------------------- 遥控器控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void RC_Controller()
{
    static enum shoot_mode_e shoot_mode_candidate = CLOSE;
    static uint8_t shoot_mode_stable_cnt = 0;
    static uint8_t single_zone_latched = 0;
    static uint8_t single_cmd_ticks = 0;

    if (RC_data.online >= 0)
        RC_data.online--;
    /*控制模式选择*/
    if ((RC_data.rc.s[0]==RC_SW_DOWN && RC_data.rc.s[1]==RC_SW_DOWN) || (RC_data.online <= 0)) // 左下右下 或 离线，锁死
        Global.Control.mode = LOCK;
    else
        Global.Control.mode = RC;
    if (Global.Control.mode == LOCK)
    {
        Global.Auto.mode = NONE;
        Global.Chassis.mode = FLOW_Chassis;
        Global.Shoot.shoot_mode = CLOSE;
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
        return;
    }
    /*底盘控制*/

    if (RC_data.rc.s[0]==RC_SW_UP && RC_data.rc.s[1]==RC_SW_MID) // 左中右上，正小陀螺
        Global.Chassis.mode = SPIN_P;
    else if (RC_data.rc.s[1]==RC_SW_DOWN && RC_data.rc.s[0]==RC_SW_MID ) // 左下右中，負小陀螺
        Global.Chassis.mode = SPIN_N;
    else if (RC_data.rc.s[1]==RC_SW_UP && RC_data.rc.s[0]==RC_SW_UP)
        Global.Chassis.mode = Navigation;
    else if (RC_data.rc.s[1]==RC_SW_MID && RC_data.rc.s[0]==RC_SW_MID)
        Global.Chassis.mode = FLOW_Gimbal;
    else
        Global.Chassis.mode = FLOW_Chassis;
    Chassis_SetX(RC_data.rc.ch[0] / 40.0f);
    Chassis_SetY(RC_data.rc.ch[1] / 40.0f);
    /*云台控制*/
    if (!AutoAim_IsControllingGimbal())
    {
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw - RC_data.rc.ch[2] / 2000.0f);
    }
    /*自瞄控制*/     
    if ((/* RC_data.rc.s[0]==RC_SW_DOWN || */ RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_UP) && RC_data.rc.s[1]==RC_SW_UP) // 右下,左中||左上，自瞄,射击模式
    {
        Global.Auto.mode = CAR;
    }
    else
    {
        Global.Auto.mode = NONE;
    }
    /*发弹机构控制*/
    enum shoot_mode_e desired_shoot_mode = CLOSE;
    if (RC_data.rc.s[1]==RC_SW_UP && (RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN || RC_data.rc.s[0]==RC_SW_UP)) // 左上，右中||右下，开启摩擦轮
        desired_shoot_mode = READY;

    if (desired_shoot_mode != shoot_mode_candidate)
    {
        shoot_mode_candidate = desired_shoot_mode;
        shoot_mode_stable_cnt = 0;
    }
    else if (shoot_mode_stable_cnt < SHOOT_MODE_DEBOUNCE_TICKS)
    {
        shoot_mode_stable_cnt++;
    }

    if (shoot_mode_stable_cnt >= SHOOT_MODE_DEBOUNCE_TICKS)
    {
        Global.Shoot.shoot_mode = desired_shoot_mode;
    }

    if (Global.Shoot.shoot_mode == CLOSE || Global.Auto.mode != NONE)
    {
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
        single_zone_latched = 0;
        single_cmd_ticks = 0;
    }
    else
    {
        if (RC_data.rc.ch[4] >= 300 && RC_data.rc.ch[4] <= 660 && (RC_data.rc.s[0] == RC_SW_UP || RC_data.rc.s[0] == RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN))
            {
                Global.Shoot.manual_trigger_mode = ONLY_AIM_HIGH;
                single_zone_latched = 0;
                single_cmd_ticks = 0;
            }
        else
        {
            if ((RC_data.rc.ch[4] >= -660 && RC_data.rc.ch[4] <= -330) && !single_zone_latched)
            {
                single_zone_latched = 1;
                single_cmd_ticks = 10;
            }
            else if (!(RC_data.rc.ch[4] >= -660 && RC_data.rc.ch[4] <= -330))
            {
                single_zone_latched = 0;
            }
            if (single_cmd_ticks > 0)
            {
                Global.Shoot.manual_trigger_mode = SINGLE;
                single_cmd_ticks--;
            }
            else
            {
                Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
            }
        }
    }
}

/*------------------------------------ Task -------------------------------------*/

/**
 * @brief          遥控任务
 * @param          none
 * @retval         none
 */
void Remote_Tasks(void)
{
	// 遥控数据更新
	RCdata_Updater();
    // 遥控器控制
    RC_Controller();
}