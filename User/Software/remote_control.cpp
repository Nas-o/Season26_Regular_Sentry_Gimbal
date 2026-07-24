/**
 * @file  remote_control.cpp
 * @brief 遥控数据汇总与控制逻辑
 * @note  支持DT7遥控器 / VT13图传 / FSI6X航模遥控器 三种输入源,
 *        统一映射到 RC_data 结构体, 再由 RC_Controller() 解析为全局控制指令
 */
#include "remote_control.hpp"
#include "Global_status.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "ramp_generator.hpp"
#include "referee_system.hpp"
#include "Auto_control.hpp"

#include "DT7.hpp"
#include "VT13.hpp"
#include "FSI6X.hpp"
#include "Power_switch.hpp"
#include "IMU_updata.hpp"

#include "Stm32_time.hpp"
#include "Power_switch.hpp"

#include "cmsis_os2.h"

#define SHOOT_MODE_DEBOUNCE_TICKS 10  // 射击模式切换去抖拍数 (10ms)


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
    // 档位与拨杆映射
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
 * @brief 根据数据源优先级更新遥控数据
 * @note  优先级: DT7 > FSI6X > VT13 (根据在线状态自动切换)
 */
void RCdata_Updater()
{
    if (DT7_data.online >= 0)      // DT7在线: 最高优先级
      DT7toRCdata();
    else if (FSI6X_data.online >= 0) // FSI6X在线
      FSI6XtoRCdata();
    else if (VT13_data.online >= 0)  // VT13在线
      VT13toRCdata(); 
}


/*----------------------------------- 遥控器控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制状态机
 * @note  根据拨杆组合决定:
 *        - 控制模式 (LOCK/RC)
 *        - 底盘模式 (FLOW_Chassis/FLOW_Gimbal/SPIN_P/SPIN_N/Navigation)
 *        - 自瞄开关 (NONE/CAR)
 *        - 射击模式 (CLOSE/READY) + 拨弹模式
 *        并包含射击模式去抖动和单发边缘检测逻辑
 */
void RC_Controller()
{
    static enum ShootMode s_shoot_mode_candidate = CLOSE;  // 射击模式候选值
    static uint8_t s_shoot_mode_stable_cnt = 0;            // 去抖计数器
    static uint8_t s_single_zone_latched = 0;              // 单发区域锁定标志
    static uint8_t s_single_cmd_ticks = 0;                 // 单发指令持续拍数

    if (RC_data.online >= 0)
        RC_data.online--;    // 在线计时器递减, 超时则判定离线

    /*=== 1. 控制模式选择 ===*/
    // 双拨杆同时下拨 或 遥控器离线 → 全机锁止
    if ((RC_data.rc.s[0]==RC_SW_DOWN && RC_data.rc.s[1]==RC_SW_DOWN) || (RC_data.online <= 0))
        Global.Control.mode = LOCK;
    else
        Global.Control.mode = RC;
    // LOCK模式: 清除所有控制状态并立即返回
    if (Global.Control.mode == LOCK)
    {
        Global.Auto.mode = NONE;
        Global.Chassis.mode = FLOW_Chassis;
        Global.Shoot.shoot_mode = CLOSE;
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
        return;
    }
    /*=== 2. 底盘模式选择 ===*/
    if (RC_data.rc.s[0]==RC_SW_UP && RC_data.rc.s[1]==RC_SW_MID)        // 右上左中: 正小陀螺
        Global.Chassis.mode = SPIN_P;
    else if (RC_data.rc.s[1]==RC_SW_DOWN && RC_data.rc.s[0]==RC_SW_MID) // 左下右中: 负小陀螺
        Global.Chassis.mode = SPIN_N;
    else if (RC_data.rc.s[1]==RC_SW_UP && RC_data.rc.s[0]==RC_SW_UP)    // 双上: 导航模式
        Global.Chassis.mode = Navigation;
    else if (RC_data.rc.s[1]==RC_SW_MID && RC_data.rc.s[0]==RC_SW_MID)  // 双中: 云台跟随
        Global.Chassis.mode = FLOW_Gimbal;
    else                                                                // 其他: 底盘跟随
        Global.Chassis.mode = FLOW_Chassis;
    Chassis_SetX(RC_data.rc.ch[0] / 40.0f);  // X轴速度指令
    Chassis_SetY(RC_data.rc.ch[1] / 40.0f);  // Y轴速度指令
    /*=== 3. 云台控制 ===*/
    // 非自瞄控制时, 手动操作云台
    if (!AutoAim_IsControllingGimbal())
    {
        // ch[3]: Pitch轴摇杆, ch[2]: Yaw轴摇杆
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw - RC_data.rc.ch[2] / 2000.0f);
    }
    /*=== 4. 自瞄控制 ===*/
    // 右拨杆上 + 左拨杆中/上 → 开启自瞄
    if ((/* RC_data.rc.s[0]==RC_SW_DOWN || */ RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_UP) && RC_data.rc.s[1]==RC_SW_UP)
    {
        Global.Auto.mode = CAR;  // 自瞄跟踪模式
    }
    else
    {
        Global.Auto.mode = NONE; // 关闭自瞄
    }
    /*=== 5. 发弹机构控制 ===*/
    // 摩擦轮开启条件: 左拨杆上 + 右拨杆中/下/上
    enum ShootMode desired_shoot_mode = CLOSE;
    if (RC_data.rc.s[1]==RC_SW_UP && (RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN || RC_data.rc.s[0]==RC_SW_UP))
        desired_shoot_mode = READY;

    // 去抖动: 连续 SHOOT_MODE_DEBOUNCE_TICKS 拍同一状态才确认切换
    if (desired_shoot_mode != s_shoot_mode_candidate)
    {
        s_shoot_mode_candidate = desired_shoot_mode;
        s_shoot_mode_stable_cnt = 0;
    }
    else if (s_shoot_mode_stable_cnt < SHOOT_MODE_DEBOUNCE_TICKS)
    {
        s_shoot_mode_stable_cnt++;
    }

    if (s_shoot_mode_stable_cnt >= SHOOT_MODE_DEBOUNCE_TICKS)
    {
        Global.Shoot.shoot_mode = desired_shoot_mode;
    }

    // 拨弹模式选择 (仅手动模式下生效)
    if (Global.Shoot.shoot_mode == CLOSE || Global.Auto.mode != NONE)
    {
        // 摩擦轮关闭或自瞄模式: 禁止手动拨弹
        Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;
        s_single_zone_latched = 0;
        s_single_cmd_ticks = 0;
    }
    else
    {
        // ch[4] 拨轮值 [300,660]: 连续高频射击
        if (RC_data.rc.ch[4] >= 300 && RC_data.rc.ch[4] <= 660 && (RC_data.rc.s[0] == RC_SW_UP || RC_data.rc.s[0] == RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN))
            {
                Global.Shoot.manual_trigger_mode = ONLY_AIM_HIGH;
                s_single_zone_latched = 0;
                s_single_cmd_ticks = 0;
            }
        else
        {
            // ch[4] 拨轮值 [-660,-330]: 单发边缘检测 (进入区域发发)
            if ((RC_data.rc.ch[4] >= -660 && RC_data.rc.ch[4] <= -330) && !s_single_zone_latched)
            {
                s_single_zone_latched = 1;   // 锁定: 避免重复触发
                s_single_cmd_ticks = 10;     // 单发指令持续10拍
            }
            else if (!(RC_data.rc.ch[4] >= -660 && RC_data.rc.ch[4] <= -330))
            {
                s_single_zone_latched = 0;   // 离开区域: 解锁
            }
            if (s_single_cmd_ticks > 0)
            {
                Global.Shoot.manual_trigger_mode = SINGLE;  // 单发中
                s_single_cmd_ticks--;
            }
            else
            {
                Global.Shoot.manual_trigger_mode = TRIGGER_CLOSE;  // 空闲
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