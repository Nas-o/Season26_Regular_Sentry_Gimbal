#ifndef __GLOBAL_STATUS_H__
#define __GLOBAL_STATUS_H__

#include "stdint.h"
typedef struct
{
    /*底盘状态*/
    struct
    {
        enum chassis_mode_e
        {
            FLOW_Gimbal,   // 底盘跟随
            SPIN_P, // 正小陀螺
            SPIN_N, // 逆小陀螺
            Navigation, //导航模式
            FLOW_Chassis,
        } mode;
        struct
        {
            float x; // x轴移动速度
            float y; // y轴移动速度
            float r; // 转速 顺时针正，逆时针负
            uint8_t reset;
        } input;
    } Chassis;
    /*云台状态*/
    struct
    {
        struct
        {
            float pitch;
            float yaw;
        } input;
        enum pitch_mode_e
        {
            Pitch_disability, // pitch电机失能
            Pitch_ability,   // pitch电机使能成功
        } pitch_mode;  
        enum small_pitch_mode_e
        {
            level,
            rush,
            outpost,
            base,
            smallPitch_target_num,
        } small_pitch_target;

        float pitch_offset;
        float small_pitch_offset;
        float Hanging_Shot_err;
        enum pitch_lock_e
        {
            Pitch_move,
            Pitch_lock,
        } pitch_lock;  

        enum yaw_lock_e
        {
            Yay_move,
            Yaw_lock,
        } yaw_lock; 

    } Gimbal;
    /*发射机构状态*/
    struct
    {
        enum trigger_mode_e
        {
            TRIGGER_CLOSE,
            HIGH,
            MID,
            LOW,
            SINGLE,
            DEBUG_TRIGGER, // 退弹使用，低射速高射频。
            ONLY_AIM_HIGH,//5hz
            ONLY_AIM_MID,//3hz
            ONLY_AIM_LOW,//1hz
        } trigger_mode;
         struct
        {
            float speed_set;
            float ecd_set;
            float ecd_last;
        } trigger;
        enum shoot_mode_e
        {
            CLOSE,
            READY,
            DEBUG_SHOOT, // 退弹使用
        } shoot_mode;
        int16_t shoot_deviation;//射速误差

        enum shoot_fire_e
        {
            STOP,
            FIRE,
        } fire_status;// 发射Hanging_Shot,// 吊射模式
        enum Hanging_Shot_e
        {
            Hanging_Shot_CLOSE,
            Hanging_Shot_OPEN,
        } Hanging_Shot;
        enum glass_mode_e
        {
            Glass_close,
            Glass_open,
        } glass_mode;
        
        enum ONtigger_e
        {
            tigger_close,// 拨弹盘泄力
            tigger_open,
        } ONtigger;
        struct
        {
            float pitch_angle;
            float yaw_angle;
        } shoot_angle;
        enum shoot_status_e
        {
            NOK,
            OK,
        } shoot_status;
        uint8_t mcu_fire_permit;              // 自瞄路径发射准入 (Auto_control.c 写)
        enum trigger_mode_e manual_trigger_mode; // 纯手动档下想打什么档位 (remote_control.c 写)
    } Shoot;
    /*自瞄状态*/
    struct
    {
        enum auto_mode_e
        {
            NONE,       // 不启用自瞄
            CAR,        // 自瞄跟踪
        } mode;
        struct
        {
            float shoot_yaw;             // 目标yaw (rad, 世界坐标系绝对值)
            float shoot_pitch;           // 目标pitch (rad, 世界坐标系绝对值)
            float yaw_ff;                // yaw速度前馈 (rad/s)
            float pitch_ff;              // pitch速度前馈 (rad/s)
            float yaw_acc_ff;            // yaw加速度前馈 (rad/s?)
            float pitch_acc_ff;          // pitch加速度前馈 (rad/s?)
            uint8_t target_id;           // 目标ID
        } input;
    } Auto;
    /*电容状态*/
    struct
    {
        enum cap_mode_e
        {
            Not_FULL, // 暂时不知道放啥
            FULL,
        } mode;
    } Cap;
    /*控制模式*/
    struct
    {
        enum control_mode_e
        {
            RC,   // 运行
            LOCK, // 锁死
        } mode;
    } Control;
} GlobalStatus_t;

extern GlobalStatus_t Global;
extern enum chassis_mode_e next_chassis_mode;
#endif