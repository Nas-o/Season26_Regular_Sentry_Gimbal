#ifndef __GLOBAL_STATUS_H__
#define __GLOBAL_STATUS_H__

#include "stdint.h"
#define A_BULLET_ECD -26219
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
        enum gimbal_mode_e
        {
            NORMAL, // 普通模式
            AUTO,   // 自瞄模式
        } mode;
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
            smallPitch_target_num,//small_pitch_target这个枚举内成员的数量（把他放到最后一个，就表示这个枚举内成员的数量）
        } small_pitch_target;

        float pitch_offset;
        float small_pitch_offset;//射速误差
        float Hanging_Shot_err;
        enum pitch_lock_e
        {
            Pitch_move, // pitch动
            Pitch_lock,   // pitch锁死
        } pitch_lock;  

        enum yaw_lock_e
        {
            Yay_move, // pitch动
            Yaw_lock,   // pitch锁死
        } yaw_lock; 

    } Gimbal;
    /*发射机构状态*/
    struct
    {
        enum tigger_mode_e
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
        } tigger_mode;
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
    } Shoot;
    /*自瞄状态*/
    struct
    {
        enum auto_mode_e
        {
            NONE,       // 不启用自瞄
            CAR,        // 车
            OUTPOST,    // 前哨站
            LOWTARGET,  // 小符
            HIGHTARGET, // 大符
        } mode;
        struct
        {
            float shoot_yaw;
            float shoot_pitch;
            float yaw_ff;            // yaw速度前馈 (rad/s)
            float pitch_ff;          // pitch速度前馈 (rad/s)
            int Auto_control_online; // 自瞄在线
            int8_t fire;             // -1:自瞄在线但未发现目标，0：发现目标，1：允许开火
            uint8_t target_id;       // 目标ID
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
            RC,   // 遥控器
            KEY,  // 键鼠
            LOCK, // 锁死
        } mode;
    } Control;
} GlobalStatus_t;

extern GlobalStatus_t Global;
extern enum chassis_mode_e next_chassis_mode;
#endif