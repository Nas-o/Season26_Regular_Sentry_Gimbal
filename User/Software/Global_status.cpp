/**
 * @file  Global_status.cpp
 * @brief 全局状态机实例化
 * @note  Global 为全局唯一的状态结构体, 各模块通过它进行通信和模式协调
 */
#include "Global_status.hpp"

GlobalStatus  Global;               // 全局状态机唯一实例
ChassisMode next_chassis_mode;      // 底盘模式切换缓冲 (用于跨帧切换)

