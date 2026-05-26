/**
 * @file UART_data_txrx.c
 * @author sethome
 * @brief 串口数据发送接受
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022 sethome
 *
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "fifo.hpp"
#include "hipnuc_dec.hpp" 
#include "UART_data_txrx.hpp"

#include "DT7.hpp"
#include "VT13.hpp"
#include "IMU_updata.hpp"
#include "hi229_imu.hpp"
#include "referee_system.hpp"
#include "motor.hpp"
#include "stp_23.hpp"
#include "CRC8_CRC16.hpp"

#include "Global_status.hpp"
#include "Auto_control.hpp"
#include "vofa+.hpp"
Navigation_data_t Navigation_receive_1;

volatile uint32_t g_uart7_sticky_count = 0; // 统计粘包次数
uint32_t g_uart7_sticky_events = 0;    // 发生粘包的接收次数
uint32_t g_uart7_extra_packets = 0;     // 额外多出来的包总数
uint32_t g_uart7_buf_overflow_err = 0;  // 缓冲区满导致的溢出风险
int Navigation_online;
// DMA控制变量
extern DMA_HandleTypeDef hdma_uart5_rx; // 遥控器，仅用接受
extern DMA_HandleTypeDef hdma_uart7_rx; // 串口7，连接电源管理模块
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart10_rx; // 串口10，连接图传模块
extern DMA_HandleTypeDef hdma_usart10_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;//串口1，连接视觉小电脑
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

// 串口控制变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5; // 遥控器，可能用不到
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;

// 将上述串口+DMA整合，并包含缓冲区
transmit_data UART1_data;
transmit_data UART2_data;
transmit_data UART3_data;
transmit_data UART5_data;
transmit_data UART7_data;
transmit_data UART10_data;

/**
 * @brief 串口初始化
 *
 * @return * void
 */
void Uart_Init(void)
{
  Uart_DMARxTxStart(&UART1_data, &huart1, &hdma_usart1_rx, &hdma_usart1_rx);
  Uart_DMARxTxStart(&UART2_data, &huart2, &hdma_usart2_rx, &hdma_usart2_rx);
  Uart_DMARxTxStart(&UART3_data, &huart3, &hdma_usart3_rx, &hdma_usart3_rx);
  Uart_DMARxTxStart(&UART5_data, &huart5, &hdma_uart5_rx, &hdma_uart5_rx);
  Uart_DMARxTxStart(&UART7_data, &huart7, &hdma_uart7_rx, &hdma_uart7_tx);
  Uart_DMARxTxStart(&UART10_data, &huart10, &hdma_usart10_rx, &hdma_usart10_tx);
}

/**
 * @brief DMA，串口中断启动，ヾ(?ω?`)o温馨提示，可以在头文件里用宏自定义串口缓冲区大小，
 * @param data 串口整合包指针
 * @param huart 串口指针
 * @param hdma_usart_rx 串口接受dma指针
 * @param hdma_usart_tx 串口发送dma指针
 */
void Uart_DMARxTxStart(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx)
{
  data->huart = huart;                 // 串口控制变量
  data->hdma_usart_rx = hdma_usart_rx; // DMA接收缓冲
  data->hdma_usart_tx = hdma_usart_tx; // DMA发送缓冲

  HAL_UARTEx_ReceiveToIdle_DMA(huart, data->rev_data, UART_BUFFER_SIZE); // 开启DMA批量数据接受
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);                        // 关闭接受过半中断
}

/**
 * @brief 串口发送数据
 *
 * @param uart 发送串口整合包
 * @param data 发送数据（数据别释放了，不然后面收不到）
 * @param size 数据大小
 */
void UART_SendData(transmit_data uart, uint8_t data[], uint16_t size)
{
  //+++++++++++++++//while(HAL_DMA_GetState(UART6_data.hdma_usart_tx) != HAL_DMA_STATE_READY)
  HAL_UART_Transmit_DMA(uart.huart, data, size); // 套娃ヾ(?ω?`)o
}

/**
?* @brief 串口接受空回调函数，用于接受不定长数据，放置数据处理函数
?* @note ?该函数为HAL库中断函数，无需在主函数中调用
?* @param huart 发生中断的串口句柄
?* @param Size ?接收到的数据长度
?*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // huart->Instance 指向触发中断的硬件串口(如 USART1, UART5)
  if (huart->Instance == USART1) // 自瞄数据//导航数据
  {
    // 解决粘包和错位问题：遍历缓冲区寻找所有可能的帧头
    // if (Size >= 20)
    // { 
    //     for (uint16_t i = 0; i <= Size - 20; i++)
    //     {
    //         if (UART1_data.rev_data[i] == 0xAA)
    //         {
    //             // 找到帧头，尝试解析。decodeNAVdata 内部包含 CRC 校验，如果通过则更新数据
    //             decodeNAVdata(&Navigation_receive_1, &UART1_data.rev_data[i], 20);
    //             Navigation_online = NAV_ONLINE_TIMEOUT_MS;
    //         }
    //     }
    // }
/*     if (UART1_data.rev_data[0] == 0xA5) // 帧头校验
    {
      Global.Auto.input.Auto_control_online = 20; // 更新在线状态
      decodeMINIPCdata(&fromMINIPC, UART1_data.rev_data, Size);
      MINIPC_to_STM32();
    } */

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
  }
  else if (huart->Instance == USART2)
  {
    /* 处理HI229 IMU数据 */
    HI229_UART_RxCpltCallback(huart, Size);
    // 重新启动DMA接收
    /* HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); */ // 关闭半传输中断
  }
  else if (huart->Instance == USART3)
  {
    /* 处理HI229 IMU数据 */
    HI229_UART_RxCpltCallback(huart, Size);
    // 重新启动DMA接收
    /* HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); */ // 关闭半传输中断
  }
  else if (huart->Instance == UART5) // 遥控器
  {
    DT7_DecodeData(UART5_data.rev_data);
   /*富斯遥控器  
   if( UART5_data.rev_data[23] != 12 ) 
    {
      FSI6X_DecodeData(UART5_data.rev_data, &FSI6X_data);
    } 
    */

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
  }
  else if (huart->Instance == UART7) 
  {
   /*  fifo_s_puts(&referee_fifo, (char *)UART7_data.rev_data, (int)Size); */
    // for (uint16_t i = 0; i < Size; i++)
    //     {
    //         /*  喂数据给解码器 */
    //         if (hipnuc_input(&hipnuc_dev, UART7_data.rev_data[i]) == 1) //
    //         {
    //           UploadData_vofa(hipnuc_dev.hi91.pitch,hipnuc_dev.hi91.roll,hipnuc_dev.hi91.yaw,0);
    //         }
    //     }
    // --- 动态解析开始 ---
            // 只要接收到的总长度大于等于结构体长度，就尝试解析
            if (Size >= NAV_FRAME_SIZE)
            { 
                for (uint16_t i = 0; i <= Size - NAV_FRAME_SIZE; )
                {
                    // 检查帧头
                    if (UART7_data.rev_data[i] == 0xAA)
                    {
                        // 传入自动计算的长度 NAV_FRAME_SIZE
                        if (decodeNAVdata(&Navigation_receive_1, &UART7_data.rev_data[i], NAV_FRAME_SIZE))
                        {
                            Navigation_online = NAV_ONLINE_TIMEOUT_MS;
                            // 核心优化：成功解析后直接跳过整帧，防止对 float 里的数据误判
                            i += NAV_FRAME_SIZE;
                            continue;
                        }
                    }
                    i++; // 未找到或校验失败，向后滑动 1 字节
                }
            }
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
  }
  else if (huart->Instance == USART10) // 图传链路裁判系统 -> STP-23协议
  {
/*     if (Size == 21)
    {
      VT13_DataSolve(UART10_data.rev_data, &VT13_data);
    } */

   /* STP-23 是字节流协议。
       DMA接收到的是一串数据（Buffer），长度为 Size。
       我们需要遍历 Buffer，把每个字节传给状态机处理。
    */
    for (uint16_t i = 0; i < Size; i++)
    {
        STP23_ProcessByte(UART10_data.rev_data[i]);
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
  }
  else
  {
    // 可选：处理未知的 huart 实例
  }
}
/**
 * @brief 串口接受完成中断回调函数，用于接受定长数据
 *
 * @param huart 接受串口号
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

// 发生错误重启串口
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart7)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart10)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart5)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  else if (huart == &huart1)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}
// end of file
