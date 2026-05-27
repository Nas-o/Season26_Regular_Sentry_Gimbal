/*
 * @Author: hao hao@qlu.edu.cn
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-21 03:57:47
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\BSP\CAN_receive_send.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_RECEIVE_SEND_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __CAN_RECEIVE_SEND_H__

//#include "cover_headerfile_h.h"
#include "fdcan.h"
extern FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus);

extern void Can_Init(void);

extern uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan,	FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf);

extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);


#ifdef __cplusplus
}
#endif
#endif /* __CAN_RECEIVE_SEND_H__ */






