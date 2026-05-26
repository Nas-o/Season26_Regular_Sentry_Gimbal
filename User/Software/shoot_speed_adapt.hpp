#ifndef __SHOOT_SPEED_ADAPT_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __SHOOT_SPEED_ADAPT_H__

#include "main.h"

/* 射速自适应参数 */
#define SPEED_QUEUE_SIZE      10       // 采集队列深度（颗）
#define SPEED_HIGH_THRESH     24.0f    // 超射速阈值 (m/s)
#define SPEED_LOW_THRESH      23.0f    // 掉速阈值 (m/s)
#define SPEED_ADJUST_STEP     50       // 单次调整步长 (RPM)
#define SPEED_OFFSET_MAX      1000     // 偏移量上限 (RPM)
#define SPEED_OFFSET_MIN      (-1000)  // 偏移量下限 (RPM)

void    ShootSpeedAdapt_Init(void);    // 初始化（读取Flash中的历史偏移）
void    ShootSpeedAdapt_Update(void);  // 周期调用（检测新弹并调整）
int16_t ShootSpeedAdapt_GetOffset(void); // 获取当前转速偏移量 (RPM)


#ifdef __cplusplus
}
#endif
#endif /* __SHOOT_SPEED_ADAPT_H__ */
