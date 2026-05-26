/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief can bsp层发送与接受
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright Copyright (c) 2024
 *
 */
// #include "cover_headerfile_h.h"
#include "CAN_receive_send.hpp"
#include "motor.hpp"
#include "dm_imu.hpp"
#include "supercup.hpp"
#include "Gimbal.hpp"
#include "Chassis.hpp"
#include "Auto_control.hpp"

// CAN寄存器及控制器
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3; // 定义原型在fdcan.c文件



/**
 * @brief 获取指定CAN总线的句柄
 */
FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus) {
    switch (can_bus) {
        case 0: return &hfdcan1;
        case 1: return &hfdcan2;
        case 2: return &hfdcan3;
        default: return &hfdcan1;
    }
}

/**
 * @brief 初始化can,包含过滤器配置与使能
 *
 */
void Can_Init(void)
{
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 将上述配置到CAN1
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan3);
}

/**
************************************************************************
* @brief:      	Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;          // 发送数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
  TxHeader.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
    return 1; // 发送*
  return 0;
}

/**
************************************************************************
* @brief:      	Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, fdcan_RxHeader, buf) != HAL_OK)
    return 0; // 接收数据
  return fdcan_RxHeader->DataLength >> 16;
}

/**
 * @brief CAN接受回调函数
 *
 * @param hfdcan
 * @param RxFifo0ITs
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  FDCAN_RxHeaderTypeDef rx_header; // CAN 数据指针
  uint8_t rx_data[8];              // 获取到的数据

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
	 
    if (Gimbal_CAN_Dispatch(rx_header.Identifier, rx_data))
    return;  // 双板消息处理完即退出，不再走电机解码
    // 超电帧
    if ((rx_header.Identifier == Supercap_receive_id) ||
        (rx_header.Identifier == Supercap_chassis_power_id))
      Supercup_DecodeCandata(hfdcan, rx_data,rx_header.Identifier);
    //IMU帧
     if(hfdcan == imu_gimbal.can_handle && rx_header.Identifier == imu_gimbal.mst_id)
      IMU_UpdateData(rx_data, &imu_gimbal);
     else if(hfdcan == imu_chassis.can_handle && rx_header.Identifier == imu_chassis.mst_id)
      IMU_UpdateData(rx_data, &imu_chassis);
    // 电机帧
    DJIMotor_DecodeCandata(hfdcan, rx_header.Identifier, rx_data);
    DMMotor_DecodeCandata(hfdcan, rx_header.Identifier, rx_data);
    /* DMMotor124_DecodeCandata(hfdcan, rx_header.Identifier, rx_data); */
  }
}

/**
 * @brief CAN错误处理回调函数，重启相关设备
 *
 * @param hfdcan
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter); // 将上述配置到CAN
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(hfdcan);
}
