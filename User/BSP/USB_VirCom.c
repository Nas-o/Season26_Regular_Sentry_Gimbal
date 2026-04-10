/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-04-08 05:14:49
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\BSP\USB_VirCom.c
 */
/**
 * @file USB_VirCom.c
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"
#include "crc8_crc16.h"
#include "Stm32_time.h"
#include "fifo.h"

#include "Global_status.h"
#include "Auto_control.h"
#include "stm32h7xx_hal.h"

/* shadow 变量定义在 Auto_control.c，这里声明外部引用 */
extern volatile MINIPC_data_t  fromMINIPC_shadow;
extern volatile uint32_t       minipc_last_rx_tick;
extern volatile uint32_t       minipc_rx_seq;



void Vircom_Send(uint8_t data[], uint16_t len)
{
/*   if (CDC_Transmit_HS(data, len) == 1) // 判断数据是否发送
  {
    // USB忙碌数据转入缓冲区

    fifo_s_puts(&USB_send_fifo, (char *)data, (int)len);
  } */
  CDC_Transmit_HS(data, len);
}

/**
 * @brief 使用 Shadow将异步的硬件数据接收与同步的软件控制周期解耦
 * 
 * @param data 
 * @param len 
 */
void Vircom_Rev(uint8_t data[], uint16_t len)
{
/*   if(data[0]==0xff)
  {
    Global.Auto.input.Auto_control_online=1;
    decodeMINIPCdata(&fromMINIPC,data,len);
    MINIPC_to_STM32();
  } */
/*     if (data[0] == 0xAA && data[1] == 19)
    {
        if (data[18] == 0x01 || data[18] == 0x00)
        {
          decodeNAVdata(&Navigation_receive_1, data, 19);
        }
    } */
   if(data[0] == 'S' && data[1] == 'P')
   {
      // 长度校验
      if (len != sizeof(MINIPC_data_t))
          return;
      
      // CRC16校验 (对 crc16 字段之前的所有字节)
      MINIPC_data_t *pkt = (MINIPC_data_t *)data;
      int crc_len = (uint8_t *)&pkt->crc16 - (uint8_t *)pkt;
      uint16_t calc_crc = get_CRC16_check_sum(data, crc_len, 0xFFFF);
      if (calc_crc != pkt->crc16)
          return;
      
      // 仅写 shadow + 时间戳 + 序列号，ISR 不再触碰任何运行态控制变量
      decodeMINIPCdata((MINIPC_data_t *)&fromMINIPC_shadow, data, len);
      minipc_last_rx_tick = HAL_GetTick();   //配合时间戳做“掉线保护”
      minipc_rx_seq++;
   }
}

#include "stdio.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  Vircom_Send((uint8_t *)&ch, 1);

  return ch;
}