/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-25 19:56:47
 * @FilePath: \Season-26-Code\User\Software\remote_control.h
 */
#ifndef __REMOTE_CONTROL__

#ifdef __cplusplus
extern "C" {
#endif

#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

typedef PACKED_STRUCT()
{
    struct
    {
        int16_t ch[5]; // 摇杆数据
        char s[2];     // 拨杆数据
    } rc;
    struct
    {
        int16_t x;         // x轴移动速度
        int16_t y;         // y轴移动速度
        int16_t z;         // z轴移动速度
        uint8_t press_l;   // 左键是否按下
        uint8_t press_r;   // 右键是否按下
        uint8_t press_mid; // 中键是否按下
    } mouse;
    struct
    {
        uint16_t v; // 键盘按键数据
    } key;
    struct
    {
        uint8_t custom_data[30]; // 自定义控制器数据
    } custom_robot;
    enum Sreial_mode_e
    {
        Sreial_RC,  // 遥控器
        Sreial_KEY, // 键鼠
    } Sreial_mode;
    int online;
}
RC_ctrl_t;

/*外部函数调用*/
void Remote_Tasks(void);
extern RC_ctrl_t RC_data;



#ifdef __cplusplus
}
#endif
#endif // !__REMOTE_CONTROL__
