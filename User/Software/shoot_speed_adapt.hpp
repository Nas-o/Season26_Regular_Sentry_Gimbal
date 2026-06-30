#ifndef __SHOOT_SPEED_ADAPT_H__

#ifdef __cplusplus
extern "C" 
{
#endif

    #define __SHOOT_SPEED_ADAPT_H__

    #include "main.h"

    /* 射速自适应编译开关宏 (保留为宏) */
    #define USE_SHOOT_SPEED_ADAPT 1

    #ifdef __cplusplus
    /* ---- 射速自适应参数 (constexpr, 仅C++可见) ---- */
    constexpr uint8_t kSpeedQueueSize     = 10;        // 采集队列深度（颗）
    constexpr float   kSpeedHighThresh    = 24.0f;     // 超射速阈值 (m/s)
    constexpr float   kSpeedLowThresh     = 23.0f;     // 掉速阈值 (m/s)
    constexpr int16_t kSpeedAdjustStep    = 50;        // 单次调整步长 (RPM)
    constexpr int16_t kSpeedOffsetMax     = 1000;      // 偏移量上限 (RPM)
    constexpr int16_t kSpeedOffsetMin     = (-1000);   // 偏移量下限 (RPM)
    #else
    /* ---- C兼容: 保留宏定义 ---- */
    #define SPEED_QUEUE_SIZE      10
    #define SPEED_HIGH_THRESH     24.0f
    #define SPEED_LOW_THRESH      23.0f
    #define SPEED_ADJUST_STEP     50
    #define SPEED_OFFSET_MAX      1000
    #define SPEED_OFFSET_MIN      (-1000)
    #endif

    void    ShootSpeedAdapt_Init(void);    // 初始化（读取Flash中的历史偏移）
    void    ShootSpeedAdapt_Update(void);  // 周期调用（检测新弹并调整）
    int16_t ShootSpeedAdapt_GetOffset(void); // 获取当前转速偏移量 (RPM)


#ifdef __cplusplus
}
#endif
#endif /* __SHOOT_SPEED_ADAPT_H__ */
