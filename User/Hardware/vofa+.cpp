/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-05 22:25:36
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-31 11:58:28
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\vofa+.c
 */
#include "vofa+.hpp"
#include "stdint.h"
#include "UART_data_txrx.hpp"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

char tempData[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x00,0x00,0x80,0x7F};//前十6个是数据帧

    void UploadData_vofa(float data1,float data2,float data3,float data4)
{
	static float temp[4];//float temp[15];

	// 检查串口是否处于就绪状态，防止DMA发送过程中修改缓冲区导致数据错乱
	// 如果串口忙（上一帧DMA未完成），则跳过本次发送
/* 	if (HAL_UART_GetState(UART10_data.huart) & HAL_UART_STATE_BUSY_TX)
	{
		return;
	} */

	temp[0]=data1;
	temp[1]=data2;
	temp[2]=data3;
	temp[3]=data4;
	memcpy(tempData, (uint8_t *)&temp, sizeof(temp));
	UART_SendData(UART10_data, (uint8_t *)&tempData,20);

}

void Ausart_printf(const char *fmt, ...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap,fmt);
	len = vsprintf((char*)tx_buf,fmt,ap);
	va_end(ap);
	//UART_send_data(UART1_data, tx_buf,len);
	
}
