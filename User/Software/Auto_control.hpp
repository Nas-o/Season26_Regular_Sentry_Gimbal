/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-20 04:29:54
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\Auto_control.hpp
 */
#ifndef __AUTO_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __AUTO_CONTROL_H__

#include "stdint.h"
#include "referee_system.hpp"
#include "robot_param.hpp"


#if(AUTO_TJU == 1)
/* ======================== 天大自瞄协议 ======================== */

#pragma pack(1)

/** @brief STM32 -> MiniPC (天大协议) */
typedef struct
{
    struct
    {
        uint8_t sof;         // 帧起始标志
        uint8_t crc8;        // 帧头CRC8校验
    } FrameHeader; // 2 bytes
    struct
    {
        float curr_yaw;      // 当前yaw角 (rad)
        float curr_pitch;    // 当前pitch角 (rad)
        float curr_omega;    // 当前yaw角速度 (rad/s)
        uint8_t state;       // 0=打车, 1=前哨站, 2=小符, 3=大符
        uint8_t autoaim;     // 0=关闭自瞄, 1=开启自瞄
        uint8_t enemy_color; // 0=蓝色, 1=红色
    } To_minipc_data; // 15 bytes
    struct
    {
        uint16_t crc16;      // 帧尾CRC16校验
    } FrameTailer; // 2 bytes
    uint8_t enter; // 1 byte (换行符)
} STM32_data_t;

/** @brief MiniPC -> STM32 (天大协议) */
typedef struct
{
    struct
    {
        uint8_t sof;         // 帧起始标志
        uint8_t crc8;        // 帧头CRC8校验
    } FrameHeader; // 2 bytes
    struct
    {
        float shoot_yaw;     // 目标yaw角 (rad)
        float shoot_pitch;   // 目标pitch角 (rad)
        uint8_t fire;        // 发弹信号 (1=允许开火)
        uint8_t target_id;   // 目标ID (UI显示用)
    } from_minipc_data; // 10 bytes
    struct
    {
        uint16_t crc16;      // 帧尾CRC16校验
    } FrameTailer;
} MINIPC_data_t;

#pragma pack(4)

void STM32_to_MINIPC(float yaw,float pitch,float omega);

#endif

#if(AUTO_CSU == 1)
/* ======================== 中南自瞄协议 ======================== */
#pragma pack(1)

/** @brief STM32 -> MiniPC (中南协议, 紧凑packed结构) */
typedef struct
{
    uint8_t header;   // 帧头
    uint8_t mode;     // 工作模式 (0=空闲, 1=自瞄, 2=小符, 3=大符)
    float roll;       // 当前roll角 (rad)
    float pitch;      // 当前pitch角 (rad)
    float yaw;        // 当前yaw角 (rad)
    uint8_t fill;     // 填充字节 (对齐)
    uint8_t ender;    // 帧尾
} __attribute__((packed))STM32_data_t;

/** @brief MiniPC -> STM32 (中南协议) */
typedef struct
{
    uint8_t header;   // 帧头
    uint8_t fire;     // 发弹信号 (1=允许)
    float pitch;      // 目标pitch角 (rad)
    float yaw;        // 目标yaw角 (rad)
    float distence;   // 目标距离 (m)
    uint8_t ender;    // 帧尾
} __attribute__((packed))MINIPC_data_t;
#pragma pack(4)

void STM32_to_MINIPC();

#endif

#if(AUTO_TongJi == 1)
#pragma pack(1)

/**
 * @brief 机器人运行模式 (下位机 -> 上位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum 
{
    MODE_IDLE       = 0, // 空闲
    MODE_AUTO_AIM   = 1, // 自瞄
    MODE_SMALL_BUFF = 2, // 小符
    MODE_BIG_BUFF   = 3, // 大符
    MODE_OUTPOST    = 4  // 前哨站 (保留)
} RobotMode;


/**
 * @brief 视觉控制模式 (上位机 -> 下位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum 
{
    CTRL_NO_CONTROL     = 0, // 不控制
    CTRL_AIM_ONLY       = 1, // 控制云台但不开火
    CTRL_AIM_AND_FIRE   = 2  // 控制云台且开火
} VisionControlMode;


/**
 * @brief 上位机发送给STM32的数据 (MiniPC -> STM32)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct 
{
    uint8_t header[2] ;   // 帧头 (固定值 {'S', 'P'})

    uint8_t mode;        // 控制模式 (VisionControlMode_e)

    float yaw;           // 偏航角 (rad)
    float yaw_vel;       // 偏航角速度 (rad/s)
    float yaw_acc;       // 偏航角加速度 (rad/s^2)

    float pitch;         // 俯仰角 (rad)
    float pitch_vel;     // 俯仰角速度 (rad/s)
    float pitch_acc;     // 俯仰角加速度 (rad/s^2) 
    

    //float center_yaw;    // yaw轴旋转中心
    float distance;      // 目标到相机距离

    uint16_t crc16;      // CRC16 校验
} MINIPC_data_t;

/**
 * @brief STM32发送给上位机的数据 (STM32 -> MiniPC)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct 
{
    uint8_t header[2];   // 帧头 (固定值 {'S', 'P'})

    uint8_t mode;        // 当前模式 (RobotMode_e)

    // --- IMU 姿态 (四元数) ---
    float q[4];          // w, x, y, z 顺序

    // --- 云台状态 ---
    float yaw;           // 当前偏航角 (rad)
    float yaw_vel;       // 当前偏航角速度 (rad/s)
    float pitch;         // 当前俯仰角 (rad)
    float pitch_vel;     // 当前俯仰角速度 (rad/s)

    // --- 射击状态 ---
    float bullet_speed;      // 弹速 (m/s)
    uint8_t enemy_color;
    uint16_t bullet_count;   // 子弹累计发送次数 (用于统计/丢包检测)

    uint16_t crc16;      // CRC16 校验
} STM32_data_t;



void STM32_to_MINIPC();
#pragma pack(4)
#endif

#pragma pack(1)

/**
 * @brief STM32 -> ROS导航系统 上传数据 (通过UART/USB)
 * @note  包含比赛状态、弹量、RFID等信息, 供导航决策使用
 */
typedef struct
{
    uint8_t game_type;                    // 比赛类型
    uint8_t game_progress;                // 比赛阶段 (0未开始, 4进行中, 5结算)
    uint16_t remain_hp;                   // 当前血量
    uint16_t max_hp;                      // 最大血量
    uint16_t stage_remain_time;           // 当前阶段剩余时间 (s)
    uint16_t bullet_remaining_num_17mm;   // 17mm弹丸剩余允许发射量
    uint16_t outpost_hp;                  // 己方前哨站血量
    uint16_t base_hp;                     // 己方基地血量
    uint32_t rfid_status;                 // RFID状态位 (各bit对应不同增益点)
    float contact_angle;                  // 大小yaw差角 (deg), 供导航判断是否对齐
    uint8_t is_fire;                      // 是否开火 (0=不开火, 非0=开火)
} STM32ROS_data_t;


/**
 * @brief ROS导航系统 -> STM32 下发数据 (底盘运动指令)
 * @note  帧格式: [header][payload][crc16], 通过UART接收
 */
#pragma pack(1)
typedef struct
{
    uint8_t header;              // 帧头 (固定值)
    float x_speed;               // x方向速度指令 (m/s, 机体坐标系)
    float y_speed;               // y方向速度指令 (m/s, 机体坐标系)
    float rotate;                // 旋转速度指令 (rad/s)
    float yaw_speed;             // yaw轴期望角速度 (rad/s)
    uint8_t running_state;       // 运动姿态切换 (0=正常, 1=加速冲刺...)
    uint8_t region_code;         // 指定区域编号 (11=小陀螺区域)
    uint8_t big_yaw_aligned;     // 大yaw是否已对齐 (1=对齐)
    float reserved[8];           // 预留字段
    uint16_t crc_val;            // CRC16校验 (Modbus)
} Navigation_data_t;
#pragma pack()


/**
 * @brief 哨兵自主决策信息 (通过裁判系统机间通信发送)
 * @note  位域结构, 总共32bit数据字段, 用于向云台手请求资源
 */
typedef struct
{
    uint16_t header;                                 // 帧头 (0xA5)
    uint16_t sender;                                 // 发送端机器人ID
    uint16_t receiver;                               // 接收端机器人ID (操作手客户端)

    uint32_t check_revive_status : 1;                // 请求查询: 是否已复活
    uint32_t check_exchange_revive_status : 1;       // 请求查询: 云台手是否确认复活
    uint32_t request_increase_ammo_number : 11;      // 请求补弹数量 (0~2047)
    uint32_t request_exchange_ammo_time : 4;         // 请求兑换弹丸次数 (0~15)
    uint32_t request_HP_time : 4;                    // 请求加血次数 (0~15)
    uint32_t reserved : 11;                          // 保留位
    uint16_t CRC_16;                                 // CRC16校验
} Sentry_cmd_t;

/**
 * @brief 裁判系统UART原始帧结构 (解析用中间缓冲)
 */
typedef struct
{
    uint8_t frame_header;        // 帧头 (0xA5)
    uint16_t length;             // 数据段长度
    uint8_t sequence;            // 帧序号
    uint8_t CRC8;                // 帧头CRC8校验
    uint8_t data[121];           // 数据段 (最大长度)
    referee_cmd_id_t referee_cmd_id; // 命令ID
} Referee_UART_data_t;

/**
 * @brief 裁判系统数据 (通过双板CAN通信从底盘板接收)
 * @note  底盘板解析裁判系统UART后, 通过CAN转发关键字段到云台板
 *        online字段由云台板根据CAN接收时间戳自动维护
 */
typedef struct
{
    uint8_t game_type;                  // 比赛类型 (1=7v7, 4=3v3...)
    uint8_t game_progress;              // 比赛阶段 (0未开始, 4进行中, 5结算)
    uint8_t launching_frequency;        // 发射频率限制 (Hz)
    uint16_t current_HP;                // 当前血量
    uint16_t maximum_HP;                // 最大血量
    uint16_t stage_remain_time;         // 当前阶段剩余时间 (s)
    uint16_t projectile_allowance_17mm; // 17mm弹丸允许发射量
    uint16_t outpost_HP;                // 己方前哨站血量
    uint16_t base_HP;                   // 己方基地血量
    uint32_t rfid_status;               // RFID状态位
    uint32_t initial_speed;             // 枪口初速度 (mm/s, 裁判系统原始值)
    
    uint32_t last_update_tick;          // 最后一次收到CAN帧的时间戳 (ms)
    uint8_t  online;                    // 在线标志: 1=在线, 0=离线/超时
} Referee_data_t;

#define REFEREE_CAN_TIMEOUT_MS  200  // 裁判系统CAN数据超时阈值 (ms)

/**
 * @brief 自瞄状态机 — 上位机 mode 做决策, 下位机只做链路保底+执行
 */
typedef enum {
    AIM_OFF        = 0,   // 自瞄关闭 (遥控手动)
    AIM_SCAN       = 1,   // 扫描找目标 (mode==0 或链路不新鲜)
    AIM_REACQUIRE  = 2,   // 重获目标冻结 (上升沿1拍)
    AIM_TRACK      = 3,   // 正常跟踪 (fresh && mode!=0)
} AimState_e;

/**
 * @brief 自瞄全局状态 — 唯一权威数据源
 *        上位机 mode 做语义决策, 下位机只保留链路保底+执行安全
 */
typedef struct 
{
    /* ---- 状态机 ---- */
    AimState_e state;               // 当前自瞄总状态
    uint8_t  rx_fresh;              // 链路新鲜: (now - last_rx_tick) <= timeout
    uint8_t  control_req;           // fresh && mode != CTRL_NO_CONTROL
    uint8_t  control_req_last;      // 上一拍, 用于边沿检测
    uint8_t  reacquire_freeze;      // 恢复冻结: 上升沿后跳过1拍Auto_Control
    uint8_t  is_scanning;           // 当前是否正在扫描
    uint8_t  mode_raw;              // 当前帧 mode 原始值

    /* ---- 扫描内部 ---- */
    float    scan_pitch_deg;
    float    scan_offset;           // 相对大yaw的扫描偏移 (deg)
    int8_t   scan_yaw_dir;
    uint8_t  scan_init_done;

    /* ---- 规划输出 ---- */
    float    target_yaw;
    float    target_pitch;

    /* ---- 调试量 (每拍填充, 可直接观察) ---- */
    float    dbg_yaw_target_raw_deg;
    float    dbg_yaw_cmd_deg;
    float    dbg_yaw_now_deg;
    float    dbg_yaw_ff_deg_s;
    float    dbg_encoder_relative;
    float    dbg_mcu_fire_permit;
} AutoAim_t;

#pragma pack(4)

/* ---- 协议/链路层 ---- */
void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len);
void MINIPC_to_STM32(void);
void MINIPC_SyncShadowToWorkingCopy(void);
void Navigation_Send_Message(void);
uint8_t decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len);
uint8_t RelativeAngle_IsFresh(void);

/* ---- 自瞄总入口 (freertos.c 每1ms调一次) ---- */
void AutoAim_Update(void);

/* ---- 查询接口 (外围模块只用这几个, 不自己拼条件) ---- */
uint8_t AutoAim_IsControllingGimbal(void);  // 自瞄正在控制云台 (TRACK/REACQUIRE/SCAN)
uint8_t AutoAim_IsTracking(void);           // 正在跟踪目标
uint8_t AutoAim_IsScanning(void);           // 正在扫描
uint8_t AutoAim_ShouldBigYawFollow(void);   // 大yaw应跟随小yaw

/* ---- 外部变量: 通信数据 ---- */
extern MINIPC_data_t fromMINIPC;             // 上位机 -> STM32 最新帧
extern STM32_data_t toMINIPC;                // STM32 -> 上位机 发送缓冲
extern Navigation_data_t Navigation_receive_1; // 导航系统 -> STM32 最新帧
extern STM32ROS_data_t stm32send_1;          // STM32 -> ROS导航 发送缓冲
extern int Navigation_online;                // 导航在线计数器 (>0=在线, 每帧--)
extern Sentry_cmd_t Sentry_cmd_1;            // 哨兵决策指令缓冲
extern Referee_data_t Referee;               // 裁判系统数据 (从底盘板CAN接收)

/* ---- 外部变量: 自瞄状态 ---- */
extern AutoAim_t AutoAim;                         // 自瞄全局状态机
extern volatile uint32_t relative_timeout_ms; // 对板相对角度超时阈值 (ms)
extern volatile uint32_t relative_tick;      // 最后一次收到相对角度的时刻

/* ---- 可调阈值 (定义在 Auto_control.cpp, 可通过vofa+在线调) ---- */
extern float big_yaw_ff_gain;             // 大yaw前馈增益 (0=关闭)
extern float big_yaw_ff_max_deg_s;        // 大yaw前馈限幅 (deg/s)
extern uint32_t rx_timeout_ms;            // 自瞄链路超时判定 (ms)
extern float    yaw_reject_deg;           // yaw偏差超过此值拒绝该帧 (deg)
extern float    yaw_ff_max_deg_s;         // 视觉yaw前馈限幅 (deg/s)

/* ---- 通信丢包检测 ---- */
extern uint32_t lost_pkg_cnt[4];              // 各通道丢包计数
void LostPkg_Detect(const float vals[], uint8_t n); // 检测数据是否连续相同(疑似丢包)
void LostPkg_Reset(uint8_t ch);               // 重置指定通道丢包计数

#define NAV_ONLINE_TIMEOUT_MS 200              // 导航离线判定超时 (ms)
#define NAV_FRAME_SIZE (sizeof(Navigation_data_t)) // 导航帧长度 (字节)

#ifdef __cplusplus
}
#endif
#endif

