/**
 * @file USB_VirCom.h
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022 sethome
 * 
 */

#ifdef __cplusplus
extern "C" {
#endif

#define __USB_VIR_COM_H__
#ifdef __USB_VIR_COM_H__

#include "stdint.h"
#include "stdbool.h"
void Vircom_Send(uint8_t data[],uint16_t len);
void Vircom_Rev(uint8_t data[],uint16_t len);

#ifdef __cplusplus
}
#endif
#endif
//end of file
