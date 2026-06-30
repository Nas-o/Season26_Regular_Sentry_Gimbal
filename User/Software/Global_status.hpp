#ifndef __GLOBAL_STATUS_H__

#ifdef __cplusplus
extern "C" 
{
#endif

    #define __GLOBAL_STATUS_H__

    #include "stdint.h"

    typedef enum ChassisMode
    {
        FLOW_Gimbal,   // 底盘跟随
        SPIN_P, // 正小陀螺
        SPIN_N, // 逆小陀螺
        Navigation, //导航模式
        FLOW_Chassis,
    } ChassisMode;

    typedef enum PitchMode
    {
        PITCH_DISABLED, // pitch电机失能
        PITCH_ENABLED,   // pitch电机使能成功
    } PitchMode;

    typedef enum TriggerMode
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
    } TriggerMode;

    typedef enum ShootMode
    {
        CLOSE,
        READY,
        DEBUG_SHOOT, // 退弹使用
    } ShootMode;
    
    typedef enum ShootStatus
    {
        NOK,
        OK,
    } ShootStatus;

    typedef enum AutoMode
    {
        NONE,       // 不启用自瞄
        CAR,        // 自瞄跟踪
    } AutoMode;

    typedef enum CapMode
    {
        Not_FULL, // 暂时不知道放啥
        FULL,
    } CapMode;

    typedef enum ControlMode
    {
        RC,   // 运行
        LOCK, // 锁死
    } ControlMode;

    typedef struct
    {
        /*底盘状态*/
        struct
        {
            ChassisMode mode;
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
            PitchMode pitch_mode;  

            float pitch_offset;


        } Gimbal;
        /*发射机构状态*/
        struct
        {
            TriggerMode trigger_mode;
            struct
            {
                float speed_set;
                float ecd_set;
                float ecd_last;
            } trigger;
            ShootMode shoot_mode;

            struct
            {
                float pitch_angle;
                float yaw_angle;
            } shoot_angle;
            ShootStatus shoot_status;
            uint8_t mcu_fire_permit;              // 自瞄路径发射准入 (Auto_control.c 写)
            TriggerMode manual_trigger_mode; // 纯手动档下想打什么档位 (remote_control.c 写)
        } Shoot;
        /*自瞄状态*/
        struct
        {
            AutoMode mode;
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
            CapMode mode;
        } Cap;
        /*控制模式*/
        struct
        {
            ControlMode mode;
        } Control;
    } GlobalStatus;

    extern GlobalStatus Global;
    extern ChassisMode next_chassis_mode;

#ifdef __cplusplus
}
#endif
#endif
