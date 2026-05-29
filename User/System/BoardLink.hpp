#ifndef __BOARD_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __BOARD_LINK_H__

#include "fdcan.h"
#include "stdint.h"

typedef enum
{
    BOARD_LINK_RX_UNHANDLED = 0,
    BOARD_LINK_RX_HANDLED,
} BoardLinkDispatchStatus_e;

/* ---- 双板通信 CAN ID (用于快发接口) ---- */
#define BL_ID_SPEED_XY        0x101
#define BL_ID_SPEED_R_YAW     0x102
#define BL_ID_CONTROL_MODE    0x104
#define BL_ID_IMU_ATTITUDE    0x105
#define BL_ID_IMU_GYRO        0x106
#define BL_ID_TRIGGER_MODE    0x108
#define BL_ID_SALTATION       0x109

void BoardLink_Init(void);
void BoardLink_TxStep(void);
uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8]);

/**
 * @brief  请求对指定帧进行快速重发 (边沿触发)
 * @param  can_id  要快发的帧CAN ID (使用 BL_ID_xxx 宏)
 * @note   调用后该帧会连续额外发送若干拍, 确保底盘板收到状态变化
 *         典型用法: 模式/射击状态等离散量发生变化时调用
 */
void BoardLink_RequestFastResend(uint16_t can_id);

#ifdef __cplusplus
}
#endif
#endif
