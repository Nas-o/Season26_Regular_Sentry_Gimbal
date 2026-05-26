/**
 * @file referee_system.h
 * @author Nas (1319621819@qq.com)
 * @brief 大疆robomaster通信协议v1.2.0（20260209）
 * @version 0.1
 * @date 2026-02-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef _REFEREE_SYSTEM_

#ifdef __cplusplus
extern "C" {
#endif

#define _REFEREE_SYSTEM_

#include "main.h"
#include "referee_system_protocol.hpp"
#include "fifo.hpp"

typedef struct // 自定义结构体，需在函数中赋值，然后在 FREERTOS 里调用  后续可根据需要自行添加
{
    float Initial_SPEED;                // 弹丸初速度（单位：m/s）
    uint8_t Launching_Frequency;        // 弹丸射速频率（单位：Hz）
    uint16_t Launching_SPEED_Limit;     // 发射机构射击初速度上限
    uint16_t Barrel_Heat;               // 发射机构的枪口热量
    uint16_t CooLing_Value;             // 枪口热量每秒冷却值
    uint16_t Heat_Limit;                // 枪口热量上限
    uint16_t projectile_allowance_17mm; // 17mm弹丸允许发弹量
    uint16_t Buffer_Energy;             // 缓冲能量（单位：J）
    uint16_t Chassis_Power_Limit;       // 机器人底盘功率上限
    uint16_t Outpost_HP;                //前哨战血量
    uint16_t Baseeee_HP;                //基地血量
    float rest_time;
    


    	//导航相关数据
	
	uint16_t remain_hp;                  //剩余血量
    uint8_t	game_type;                   //比赛类型
    uint8_t	game_progress;               //比赛阶段
    float	stage_remain_time;             //当前阶段剩余时间
    uint16_t	bullet_remaining_num_17mm; //剩余子弹数
    uint16_t	red_outpost_hp;            //红方前哨站血量
    uint16_t	red_base_hp;               //红方基地血量
    uint16_t	blue_outpost_hp;           //蓝方前哨站血量
    uint16_t	blue_base_hp;              //蓝方基地血量
    uint32_t rfid_status;                //蓝方RFID状态
	uint8_t    robot_id;                  //机器人ID
	uint16_t max_hp;
} REFEREE_DATA_t;

/* 字节偏移量  大小  说明  
                    bit 0-3：比赛类型 
                    ? 1：RoboMaster  机甲大师超级对抗赛 
                    ? 2：RoboMaster  机甲大师高校单项赛 
                    ? 3：ICRA RoboMaster 高校人工智能挑战赛 
                    ? 4：RoboMaster 机甲大师高校联盟赛 3V3 对抗   
                    ? 5：RoboMaster  机甲大师高校联盟赛步兵对抗 
        0       1   bit 4-7：当前比赛阶段 
                    ? 0：未开始比赛 
                    ? 1：准备阶段 
                    ? 2：十五秒裁判系统自检阶段 
                    ? 3：五秒倒计时 
                    ? 4：比赛中 
                    ? 5：比赛结算中   
////////////////////////////////////////////////////////////////
        1       2   当前阶段剩余时间，单位：秒 
////////////////////////////////////////////////////////////////  
        3       8   UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效 */

typedef PACKED_STRUCT() // 0x0001  比赛状态数据11
{
    uint8_t game_type : 4;      /**比赛类型**/
    uint8_t game_progress : 4;  /**当前比赛阶段**/
    uint16_t stage_remain_time; /*当前阶段剩余时间*/
    uint64_t SyncTimeStamp;
}
ext_game_status_t;

/***************
*****0：平局  1：红方胜利   2：蓝方胜利
***/
typedef PACKED_STRUCT() // 0x0002  比赛结果数据1
{
    uint8_t winner; /******/
}
ext_game_result_t;


/* 字节偏移量  大小  说明 
    0          2    己方 1 号英雄机器人血量，若该机器人未上场或者被罚下，则血量为 0，下文同理 
    2          2    己方 2 号工程机器人血量 
    4          2    己方 3 号步兵机器人血量 
    6          2    己方 4 号步兵机器人血量 
    8          2    保留位 
    10         2    己方 7 号哨兵机器人血量 
    12         2    己方前哨站血量 
    14         2    己方基地血量 */

typedef PACKED_STRUCT() // 0x0003  机器人血量数据
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved; 
    uint16_t ally_7_robot_HP; /*哨兵*/
    uint16_t ally_outpost_HP; /**前哨站**/
    uint16_t ally_base_HP;
}
ext_game_robot_HP_t; /**血量**/

/* 字节偏移量  大小  说明  
                    0：未占领/未激活   1：已占领/已激活 
                    bit 0-2： 
                    bit 0：己方与资源区区不重叠的补给区占领状态，1 为已占领 
                    bit 1：己方与资源区重叠的补给区占领状态，1 为已占领 
                    bit 2：己方补给区的占领状态，1 为已占领（仅  RMUL  适用）  
                    bit 3-6：己方能量机关状态 
                    bit 3-4：己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活 
                    bit 5-6：己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活  
        0       4   bit 7-8：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领 
                    bit 9-10：己方梯形高地的占领状态，1 为已占领 
                    bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0） 
                    bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标 
                    bit 23-24：中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅  RMUL  适用） 
                    bit 25-26：己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领 
                    bit 27-28：己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领 
                    bit 29：己方基地增益点的占领状态，1 为已占领 
                    bit 30-31：保留位 */
typedef PACKED_STRUCT() // 0x0101  场地事件数据
{
    uint32_t event_data;
}
ext_event_data_t;

/* 字节偏移量  大小  说明  
                    己方最后一次受到判罚的等级： 
                    1：双方黄牌 
        0       1   2：黄牌 
                    3：红牌 
                    4：判负
//////////////////////////////////////////////////////////////// 
        1       1   己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101） 判负和双方黄牌时，该值为 0 
////////////////////////////////////////////////////////////////                    
        2       1   己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。） */
typedef PACKED_STRUCT() // 0x0104  裁判警告数据
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
}
ext_referee_warning_t;

/* 字节偏移量  大小  说明 
        0       1   己方飞镖发射剩余时间，单位：秒 1  2
////////////////////////////////////////////////////////////////         
                    bit 0-2： 最近一次己方飞镖击中的目标，开局默认为  0，1  为击中前哨站，2  为击中基地固定目标，3  为击中基地随机固定目标，4  为击中基地随机移动目标，5 为击中基地末端移动目标 
        1       2   bit 3-5: 对方最近被击中的目标累计被击中计次数，开局默认为 0，至多为 4 
                    bit 6-8： 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为  0，选中基地固定目标为  1，选中基地随机固定目标为  2，选中基地随机移动目标为 3，选中基地末端移动目标为 4 
                    bit 9-15：保留 */
typedef PACKED_STRUCT() // 0x0105  飞镖发射时间数据
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
}
ext_dart_remaining_time_t;

/*
字节偏移量  大小  说明
    0       1    本机器人 ID
    1       1    机器人等级
    2       2    机器人当前血量
    4       2    机器人血量上限
    6       2    机器人枪口热量每秒冷却值
    8       2    机器人枪口热量上限
    10      2    机器人底盘功率上限
////////////////////////////////////////////////////////////////    
                 电源管理模块的输出情况：
                 bit 0： gimbal 口输出： 0 为无输出， 1 为 24V 输出
   12       1    bit 1： chassis 口输出： 0 为无输出， 1 为 24V 输出
                 bit 2： shooter 口输出： 0 为无输出， 1 为 24V 输出
                 bit 3-7： 保留
*/
typedef PACKED_STRUCT() // 0x0201  机器人性能体系数据
{

    uint8_t robot_id;                           // 本机器人 ID
    uint8_t robot_level;                        // 机器人等级
    uint16_t current_HP;                        // 机器人当前血量
    uint16_t maximum_HP;                        // 机器人血量上限
    uint16_t shooter_barrel_cooling_value;      // 机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;         // 机器人枪口热量上限
    uint16_t chassis_power_limit;               // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1; // 电源管理模块输出情况：
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
}
ext_robot_status_t;

typedef PACKED_STRUCT() // 0x0202  实时功率热量数据
{
    uint16_t reserved1;                  //保留位
    uint16_t reserved2;                  //保留位
    float reserved3;                     //保留位
    uint16_t buffer_energy;             //缓冲能量（单位：J）
    uint16_t shooter_17mm_barrel_heat;  //17mm 发射机构的射击热量   
    uint16_t shooter_42mm_barrel_heat;  //42mm 发射机构的射击热量 
}
ext_power_heat_data_t;

/*******************
0  本机器人位置 x 坐标，单位：m
4  本机器人位置 y 坐标，单位：m
8  本机器人测速模块朝向，单位：度。正北为 0 度
**************/
typedef PACKED_STRUCT() // 0x0203  机器人位置数据
{
    float x; 
    float y;   
    float angle; 
}
ext_robot_pos_t;


typedef PACKED_STRUCT() // 0x0204  机器人增益数据
{
    uint8_t recovery_buff;       //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint16_t cooling_buff;       //机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
    uint8_t defence_buff;        //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff;  //机器人负防御增益（百分比，值为 30 表示-30%防御增益
    uint16_t attack_buff;        //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）  
    uint8_t remaining_energy;    /* bit 0-6：机器人剩余能量值反馈，以 6进制标识机器人剩余能量值比例，仅在机器人剩余能量小于50%时反馈，
                                            其余默认反馈0x80。机器人初始能量视为100% 
                                    bit 0：在剩余能量≥125%时为 1，其余情况为 0 
                                    bit 1：在剩余能量≥100%时为 1，其余情况为 0 
                                    bit 2：在剩余能量≥50%时为 1，其余情况为 0 
                                    bit 3：在剩余能量≥30%时为 1，其余情况为 0 
                                    bit 4：在剩余能量≥15%时为 1，其余情况为 0 
                                    bit 5：在剩余能量≥5%时为 1，其余情况为 0 
                                    bit 6：在剩余能量≥1%时为 1，其余情况为 0 */
}
ext_buff_t;

typedef PACKED_STRUCT() // 0x0206  伤害状态数据
{
    uint8_t armor_id : 4;            // 当扣血原因为装甲模块或测速模块时，该 4bit 组成的数值为装甲模块或测速模块的 ID 编号；其他原因扣血时，该数值为
    uint8_t HP_deduction_reason : 4; /* 血量变化类型     0：装甲模块被弹丸攻击导致扣血 
                                                        1：装甲模块或超级电容管理模块离线导致扣血 
                                                        5：装甲模块受到撞击导致扣血 */ 
}
ext_hurt_data_t;

typedef PACKED_STRUCT() // 0x0207  实时射击数据
{
    uint8_t bullet_type;         // 弹丸类型： bit1：17mm 弹丸 bit2：42mm 弹丸
    uint8_t shooter_number;      // 1：17mm 发射机构 2：保留位 3：42mm 发射机构
    uint8_t launching_frequency; // 弹丸射速频率（单位：Hz）
    float initial_speed;         // 弹丸初速度（单位：m/s）
}
ext_shoot_data_t;

typedef PACKED_STRUCT() // 0x0208  允许发弹量
{
    uint16_t projectile_allowance_17mm;        //机器人自身拥有的 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;        //42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;              //剩余金币数量
    uint16_t projectile_allowance_fortress;    //堡垒增益点提供的储备 17mm 弹丸允许发弹量； 该值与机器人是否实际占领堡垒无关
}
ext_projectile_allowance_t;

/****************
bit 位值为 1/0 的含义：是否已检测到该增益点 RFID
bit 0：己方基地增益点
bit 1：己方中央高地增益点
bit 2：对方中央高地增益点 
bit 3：己方梯形高地增益点 
bit 4：对方梯形高地增益点
bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
bit 9：己方地形跨越增益点（中央高地下方）
bit 10：己方地形跨越增益点（中央高地上方）
bit 11：对方地形跨越增益点（中央高地下方）
bit 12：对方地形跨越增益点（中央高地上方）
bit 13：己方地形跨越增益点（公路下方）
bit 14：己方地形跨越增益点（公路上方）
bit 15：对方地形跨越增益点（公路下方）
bit 16：对方地形跨越增益点（公路上方）
bit 17：己方堡垒增益点
bit 18：己方前哨站增益点 
bit 19: 己方与资源区不重叠的补给区/RMUL 补给区 
bit 20：己方与资源区重叠的补给区
bit 21：己方装配增益点
bit 22：对方装配增益点
bit 23：中心增益点（仅  RMUL  适用）
bit 24：对方堡垒增益点 
bit 25：对方前哨站增益点 
bit 26：己方地形跨越增益点（隧道）（靠近己方一侧公路区下方） 
bit 27：己方地形跨越增益点（隧道）（靠近己方一侧公路区中间） 
bit 28：己方地形跨越增益点（隧道）（靠近己方一侧公路区上方） 
bit 29：己方地形跨越增益点（隧道）（靠近己方梯形高地较低处） 
bit 30：己方地形跨越增益点（隧道）（靠近己方梯形高地较中间） 
bit 31：己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
/////////////////////////////////////////////////////////////////
bit 0：对方地形跨越增益点（隧道）（靠近对方公路一侧下方） 
bit 1：对方地形跨越增益点（隧道）（靠近对方公路一侧中间） 
bit 2：对方地形跨越增益点（隧道）（靠近对方公路一侧上方） 
bit 3：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处） 
bit 4：对方地形跨越增益点（隧道）（靠近对方梯形高地较中间） 
bit 5：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
*****************/
typedef PACKED_STRUCT() // 0x0209  机器人 RFID 状态
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
}
ext_rfid_status_t;

typedef PACKED_STRUCT() // 0x020A  飞镖选手端指令数据
{
    uint8_t dart_launch_opening_status; // 当前飞镖发射站的状态： 1：关闭；2：正在开启或者关闭中；0：已经开
    uint8_t reserved4;                   //保留位
    uint16_t target_change_time;        // 切换打击目标时的比赛剩余时间，单位：s，无未切换动作默认为 0
    uint16_t latest_launch_cmd_time;    // 最后一次操作手确定发射指令时的比赛剩余时间，单位：s，初始值为 0
}
ext_dart_client_cmd_t;

/***************
0  己方英雄机器人位置 x 轴坐标，单位：m
4  己方英雄机器人位置 y 轴坐标，单位：m
8  己方工程机器人位置 x 轴坐标，单位：m
12  己方工程机器人位置 y 轴坐标，单位：m
16  己方 3 号步兵机器人位置 x 轴坐标，单位：m
20  己方 3 号步兵机器人位置 y 轴坐标，单位：m
24  己方 4 号步兵机器人位置 x 轴坐标，单位：m
28  己方 4 号步兵机器人位置 y 轴坐标，单位：m
32  保留
36  保留
注意！！！！！
场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向。
*************/
typedef PACKED_STRUCT() // 0x020B  地面机器人位置数据
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved5;       
    float reserved6;
}
ext_ground_robot_position_t; 

/* 说明                                           备注
bit 0：对方 1 号英雄机器人易伤情况 
bit 1：对方 2 号工程机器人易伤情况 
bit 2：对方 3 号步兵机器人易伤情况 
bit 3：对方 4 号步兵机器人易伤情况                                                                     
bit 4：对方空中机器人特殊标识情况                    对方机器人：在对应机器人被标记进度≥100 时发送 1，
bit 5：对方哨兵机器人易伤情况                                              被标记进度<100 时发送 0。
bit 6：己方 1 号英雄机器人特殊标识情况 
bit 7：己方 2 号工程机器人特殊标识情况               己方机器人：在对应机器人被标记进度≥50 时发送 1， 
bit 8：己方 3 号步兵机器人特殊标识情况                                     被标记进度<50 时发送 0。
bit 9：己方 4 号步兵机器人特殊标识情况 
bit 10：己方空中机器人特殊标识情况 
bit 11：己方哨兵机器人特殊标识情况 
bit 12-15：保留位 */
typedef PACKED_STRUCT() // 0x020C  雷达标记进度数据
{
    uint16_t mark_progress; 
}
ext_radar_mark_data_t;

/* bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值 
bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为  0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数 
bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为  0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数 
bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为 1，否则为 0 
bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为 1，否则为 0 
bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。 
bit 31：保留
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为  1，否则为  0 
bit 1-11：队伍  17mm  允许发弹量的剩余可兑换数 
bit 12-13：哨兵当前姿态，1 为进攻姿态，2 为防御姿态，3 为移动姿态 
bit 14：己方能量机关是否能够进入正在激活状态，1 为当前可激活 
bit 15：保留位 */
typedef PACKED_STRUCT() // 0x020D 哨兵决策指令
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
}
ext_sentry_info_t;

/* bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发双倍易伤的机会，至多为 2 
bit 2：对方是否正在被触发双倍易伤 
0：对方未被触发双倍易伤 
1：对方正在被触发双倍易伤 
bit 3-4：己方加密等级（即对方干扰波难度等级），开局为 1，最高为 3 
bit 5：当前是否可以修改密钥，1 为可修改 
bit 6-7：保留位 */
typedef PACKED_STRUCT() // 0x020E 雷达是否具有双倍易伤资格
{
    uint8_t radar_info;
}
ext_radar_info_t;

/**bit 0：哨兵机器人是否确认复活
0 表示哨兵机器人不想复活，即使此时哨兵复活读条已经
完成
1 表示哨兵机器人想复活，若复活读条完成将立即复活
bit 1：哨兵机器人是否想要兑换立即复活
0 表示哨兵机器人不想兑换立即复活；
1 表示哨兵机器人想兑换立即复活，若此时哨兵机器人符
合兑换立即复活的规则要求，则会立即消耗金币兑换哨兵
的立即复活
bit2-12：哨兵想要兑换的发弹量值，开局为 0，修改此值后，
哨兵在补血点即可兑换允许发弹量。
此值的变化需要单调递增，否则视为不合法。
示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 X，则消
耗 X 金币成功兑换 X 允许发弹量。此后哨兵可将其从 X 修改至
X+Y，以此类推。
bit 13-16：哨兵想要远程兑换发弹量的请求次数，开局为 0，
修改此值即可请求远程兑换发弹量。
此值的变化需要单调递增且每次仅能增加 1，否则视为不合
法。
示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 1，则消
耗金币远程兑换允许发弹量。此后哨兵可将其从 1 修改至 2，
以此类推。
bit 17-20：哨兵想要远程兑换血量的请求次数，开局为 0，修
改此值即可请求远程兑换血量。
此值的变化需要单调递增且每次仅能增加 1，否则视为不合
法。
示例：此值开局仅能为 0，此后哨兵可将其从 0 修改至 1，则消
耗金币远程兑换血量。此后哨兵可将其从 1 修改至 2，以此类
推。
在哨兵发送该子命令时，服务器将按照从相对低位到相对高位
的原则依次处理这些指令，直至全部成功或不能处理为止。 
示例：若队伍金币数为 0，此时哨兵战亡，
“是否确认复活”的值为 1，“是否确认兑换立即复活”的值为 1，“确认兑换的允许发弹量值”为 100。
（假定之前哨兵未兑换过允许发弹量）由于此时队伍金币数不足以使哨兵兑换立即复活，
则服务器将会忽视后续指令，等待哨兵发送的下一组指令。
bit 21-22：哨兵修改当前姿态指令，1 为进攻姿态，2 为
防御姿态，3 为移动姿态，默认为 3；修改此值即可改变
哨兵姿态。 
bit 23：哨兵机器人是否确认使能量机关进入正在激活状
态，1 为确认。默认为 0。 
bit 24-31：保留位。
**/
typedef PACKED_STRUCT() // 0x0120  哨兵自主决策指令
{
    uint32_t sentry_cmd;
}
ext_sentry_cmd_t;

/* 说明                       备注  
雷达是否确认触发双倍易伤        开局为 0，修改此值即可请求触发双倍易伤，若此时雷达拥有触发双倍易伤的机会，则可触发。
                              此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
                              示例：此值开局仅能为 0，此后雷达可将其从 0 修改至 1，
                              若雷达拥有触发双倍易伤的机会，则触发双倍易伤。
                              此后雷达可将其从 1 修改至 2，以此类推。 
                              若雷达请求双倍易伤时，双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生效。
                              
   
密钥更新或验证指令               每个字节均为 ASCII 码编码的字母或数字。开局为随机值。
                               byte1 为指令类型，byte2-7 为密钥值。 
                               当 byte1 值为 1 时，修改此值即可更新己方加密密钥；
                               当byte1 值为 2 时，修改此值即可将雷达破解的对方密钥传输给服务器以验证是否正确破解。
                               注意： 
                               仅开局和每次对方破解成功使得加密等级（己方干扰波难度）提高时可以修改密钥，其余时间修改无效。
                               当 byte1 值为 2 时，每次更新验证密钥后的 10 秒内，再次更新无效。
 */
typedef PACKED_STRUCT() // 0x0121 雷达自主决策指令
{
    uint8_t radar_cmd;     
    uint8_t password_cmd; 
    uint8_t password_1; 
    uint8_t password_2; 
    uint8_t password_3; 
    uint8_t password_4; 
    uint8_t password_5; 
    uint8_t password_6;
}
ext_radar_cmd_t;

/* 机器人交互数据通过常规链路发送，其数据段包含一个统一的数据段头结构。数据段头结构包括内容 ID、
发送者和接收者的 ID、内容数据段。机器人交互数据包的总长不超过 127 个字节，减去 frame_header、cmd_id 和 frame_tail 的 9 个字节以及数据段头结构的 6 个字节，故机器人交互数据的内容数据段最大
为 112 个字节。 
每 1000 毫秒，英雄、工程、步兵、空中机器人、飞镖能够接收数据的上限为 3720 字节，雷达和哨兵机器
人能够接收数据的上限为 5120 字节。 由于存在多个内容 ID，但整个 cmd_id 上行频率最大为 30Hz，请合理安排带宽。 */

typedef PACKED_STRUCT() // 0x0301  机器人交互数据
{
    uint16_t data_cmd_id;   // 需为开放的子内容 ID
    uint16_t sender_id;     // 发送者 ID
    uint16_t receiver_id;   // 接收者 ID
    uint8_t user_data[100]; // 内容数据段，最大为 112
}
ext_robot_interaction_data_t;

typedef PACKED_STRUCT() // 0x0303  命令码
{
    float target_position_x; // 目标位置 x 轴坐标，单位 m 当发送目标机器人 ID 时，该值为 0
    float target_position_y; // 目标位置 y 轴坐标，单位 m 当发送目标机器人 ID 时，该值为 0
    uint8_t cmd_keyboard;    // 云台手按下的键盘按键通用键值 无按键按下则为 0
    uint8_t target_robot_id; // 对方机器人 ID 当发送坐标数据时，该值为 0
    uint16_t cmd_source;      // 信息来源 ID
}
ext_map_command_t;

typedef PACKED_STRUCT() // 0x0305  选手端小地图可接收机器人数据。雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示
{
    uint16_t hero_position_x;         //英雄机器人 x 位置坐标，单位：cm
    uint16_t hero_position_y;         //英雄机器人 y 位置坐标，单位：cm 
    uint16_t engineer_position_x;     //工程机器人 x 位置坐标，单位：cm
    uint16_t engineer_position_y;     //工程机器人 y 位置坐标，单位：cm 
    uint16_t infantry_3_position_x;   //3 号步兵机器人 x 位置坐标，单位：cm
    uint16_t infantry_3_position_y;   //3 号步兵机器人 y 位置坐标，单位：cm  
    uint16_t infantry_4_position_x;   //4 号步兵机器人 x 位置坐标，单位：cm  
    uint16_t infantry_4_position_y;   //4 号步兵机器人 y 位置坐标，单位：cm  
    uint16_t reserved7;                //保留位
    uint16_t reserved8;                //保留位
    uint16_t sentry_position_x;       //哨兵机器人 x 位置坐标，单位：cm
    uint16_t sentry_position_y;       //哨兵机器人 y 位置坐标，单位：cm
                                //注意！！！哨兵机器人或半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据
                                //         该路径会在小地图上显示。
}
ext_map_robot_data_t;

typedef PACKED_STRUCT() // 0x0307  己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。
{
    uint8_t intention;               //1：到目标点攻击 2：到目标点防守 3：移动到目标点
    uint16_t start_position_x;       //路径起点 x 轴坐标，单位：dm                  小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。
    uint16_t start_position_y;       //路径起点 y 轴坐标，单位：dm                  显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示
    int8_t delta_x[49];              //路径点 x 轴增量数组，单位：dm
    int8_t delta_y[49];              //路径点 y 轴增量数组，单位：dm                增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位
    uint16_t sender_id;
}
ext_map_data_t;

typedef PACKED_STRUCT() // 0x0308  己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。
{
    uint16_t sender_id;     // 发送者ID
    uint16_t receiver_id;   // 接受者ID
    uint16_t user_data[30]; // 字符，符合UTF-16,支持中文，注意数据大小端
}
ext_custom_info_t;

/*===============================图传链路数据=====================================*/

typedef PACKED_STRUCT() // 0x0302  操作手可使用自定义控制器通过图传链路向对应的机器人发送数据
{
    uint8_t data[25];        //最高30
}
ext_custom_robot_data_t;

typedef PACKED_STRUCT()// 0x0309 机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据
{
    uint8_t data[25];        //最高30
}
ext_robot_custom_data_t; 

typedef PACKED_STRUCT() // 0x0306  自定义控制器与选手端交互数据
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved9;
}
ext_custom_client_data_t;

typedef PACKED_STRUCT()// 0x0310 机器人可以通过图传链路向自定义客户端发送自定义信息以及接受自定义客户端的自定义指令。
{
    uint8_t data[280];        //最高300
}
ext_robot_custom_data_2_t; 

typedef PACKED_STRUCT()// 0x0311 自定义客户端发送给机器人的自定义指令
{
    uint8_t data[25];        //最高30
}
ext_robot_custom_data_3_t; 

///************************** 机器人间交互数据 ********************/
///* 交互数据接收信息： 0x0301 */
typedef PACKED_STRUCT()
{
    uint16_t data_cmd_id; // 数据段的内容 ID
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* 发送数据结构体 */
typedef struct
{
    uint16_t cmd_id;
    ext_student_interactive_header_data_t data_header;
    uint8_t data[120];
    uint16_t frame_tail;
} ext_referee_send_data_t;


typedef enum
{

    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    RED_DART = 8,
    RED_RADAR = 9,
    RED_OUTPOST = 10,
    RED_LOCATION = 11,

    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_DART = 108,
    BLUE_RADAR = 109,
    BLUE_OUTPOST = 110,
    BLUE_LOCATION = 111,

} ext_robot_id_t;

typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} ext_game_progress_t;

extern REFEREE_DATA_t Referee_data;
extern fifo_s_t referee_fifo;
extern fifo_s_t referee_image_fifo;
extern unpack_data_t referee_unpack_obj;
extern unpack_data_t referee_image_unpack_obj;
extern ext_robot_status_t robot_status;  
extern ext_game_status_t game_status;
extern ext_game_robot_HP_t game_robot_HP;
extern ext_rfid_status_t rfid_status;

void Refree_system_init(void);
void Referee_unpack_fifo_data(fifo_s_t* p_fifo,unpack_data_t* p_obj);



#ifdef __cplusplus
}
#endif
#endif // !_REFEREE_SYSTEM_
