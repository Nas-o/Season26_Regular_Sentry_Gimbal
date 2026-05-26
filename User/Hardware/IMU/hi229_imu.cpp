/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-12-08 19:24:52
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2025-12-12 11:19:14
 * @FilePath: \Season-26-Code-body\User\Hardware\IMU\hi229_imu.c
 */

#include "hi229_imu.hpp"
#include "User_math.hpp"
#include <string.h>

/* 物理常量 */
#define GRAVITY_CONSTANT        9.80665f    /* 重力加速度 m/s? */


/* CRC16计算函数 */
static uint16_t hi229_crc16(uint16_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i, j;
    
    for (j = 0; j < length; j++) {
        crc ^= (uint16_t)data[j] << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/* 全局IMU实例定义（参考dm_imu.c） */
hi229_handle_t hi229_usart2 = {0};
hi229_handle_t hi229_usart3 = {0};

/* 初始化IMU模块（参考IMU_RequestData风格） */
void HI229_Init(hi229_handle_t *imu, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx)
{
    if (imu == NULL) return;
    
    /* 清零结构体 */
    memset(imu, 0, sizeof(hi229_handle_t));
    
    /* 设置硬件接口 */
    imu->huart = huart;
    imu->hdma_rx = hdma_rx;
    imu->is_initialized = 1;
}

/* 启动IMU模块 */
void HI229_Start(hi229_handle_t *imu)
{
    if (imu == NULL || !imu->is_initialized) return;
    
    /* 启动DMA接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(imu->huart, imu->rx_buffer, HI229_MAX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(imu->hdma_rx, DMA_IT_HT);
}

/* 解析IMUSOL数据包（参考IMU_UpdateData风格） */
static uint8_t hi229_parse_imusol(hi229_handle_t *imu, uint8_t *data)
{
    hi229_imusol_t *imusol = (hi229_imusol_t *)data;
    
    if (imusol->tag != HI229_TAG_IMUSOL) {
        return 0;
    }
    
    /* 保存原始数据包 */
    memcpy(&imu->raw_packet, imusol, sizeof(hi229_imusol_t));
    
    /* 设置新数据标志（时钟中断会处理） */
    imu->new_data_flag = 1;
    
    return 1;
}

/* 解析数据帧 */
static uint8_t hi229_parse_frame(hi229_handle_t *imu, uint8_t *frame, uint16_t len)
{
    uint16_t crc_calc, crc_frame;
    uint16_t payload_len;
    uint8_t *payload;
    
    /* 检查帧头 */
    if (frame[0] != HI229_SYNC1 || frame[1] != HI229_SYNC2) {
        return 0;
    }
    
    /* 获取载荷长度 */
    payload_len = frame[2] | (frame[3] << 8);
    if (payload_len > (len - HI229_HEADER_SIZE)) {
        return 0;
    }
    
    /* 校验CRC */
    crc_frame = frame[4] | (frame[5] << 8);
    crc_calc = hi229_crc16(0, frame, 4);
    payload = frame + HI229_HEADER_SIZE;
    crc_calc = hi229_crc16(crc_calc, payload, payload_len);
    
    if (crc_calc != crc_frame) {
        return 0;
    }
    
    /* 查找IMUSOL数据包 */
    for (uint16_t offset = 0; offset < payload_len; offset++) {
        if (payload[offset] == HI229_TAG_IMUSOL) {
            if ((offset + sizeof(hi229_imusol_t)) <= payload_len) {
                return hi229_parse_imusol(imu, &payload[offset]);
            }
            break;
        }
    }
    
    return 0;
}

/* 字节输入处理（状态机） */
static void hi229_process_byte(hi229_handle_t *imu, uint8_t byte)
{
    switch (imu->parse_state) {
        case 0: /* IDLE */
            if (byte == HI229_SYNC1) {
                imu->raw_buffer[0] = byte;
                imu->raw_index = 1;
                imu->parse_state = 1;
            }
            break;
            
        case 1: /* SYNC1 */
            if (byte == HI229_SYNC2) {
                imu->raw_buffer[1] = byte;
                imu->raw_index = 2;
                imu->parse_state = 2;
            } else {
                imu->parse_state = 0;
            }
            break;
            
        case 2: /* SYNC2 - 长度低字节 */
            imu->raw_buffer[2] = byte;
            imu->raw_index = 3;
            imu->parse_state = 3;
            break;
            
        case 3: /* 长度高字节 */
            imu->raw_buffer[3] = byte;
            imu->frame_len = (imu->raw_buffer[3] << 8) | imu->raw_buffer[2];
            
            if (imu->frame_len > (HI229_MAX_FRAME_SIZE - HI229_HEADER_SIZE)) {
                imu->parse_state = 0;
                break;
            }
            
            imu->raw_index = 4;
            imu->parse_state = 4;
            break;
            
        case 4: /* CRC低字节 */
            imu->raw_buffer[4] = byte;
            imu->raw_index = 5;
            imu->parse_state = 5;
            break;
            
        case 5: /* CRC高字节 */
            imu->raw_buffer[5] = byte;
            imu->raw_index = 6;
            imu->parse_state = 6;
            break;
            
        case 6: /* 数据载荷 */
            imu->raw_buffer[imu->raw_index++] = byte;
            
            /* 检查是否接收完整个帧 */
            if (imu->raw_index >= (imu->frame_len + HI229_HEADER_SIZE)) {
                /* 解析帧 */
                hi229_parse_frame(imu, imu->raw_buffer, imu->raw_index);
                
                /* 重置状态 */
                imu->parse_state = 0;
                imu->raw_index = 0;
            }
            break;
    }
}

/* ====================== 时钟中断调用函数 ====================== */

/* 更新IMU数据（在时钟中断中调用，参考main.c中IMU数据处理） */
static void hi229_update_imu_data(hi229_handle_t *imu)
{
    if (imu == NULL || !imu->is_initialized) return;
    
    /* 检查是否有新数据需要处理 */
    if (imu->new_data_flag) {
        /* 更新陀螺仪数据（deg/s转rad/s） */
        imu->imu_data.gyro[0] = imu->raw_packet.gyro[0] * DEG_TO_RAD;
        imu->imu_data.gyro[1] = imu->raw_packet.gyro[1] * DEG_TO_RAD;
        imu->imu_data.gyro[2] = imu->raw_packet.gyro[2] * DEG_TO_RAD;
        
        /* 更新加速度数据（G转m/s?） */
        imu->imu_data.accel[0] = imu->raw_packet.acc[0] * GRAVITY_CONSTANT;
        imu->imu_data.accel[1] = imu->raw_packet.acc[1] * GRAVITY_CONSTANT;
        imu->imu_data.accel[2] = imu->raw_packet.acc[2] * GRAVITY_CONSTANT;
        
        /* 更新欧拉角数据（deg，直接使用） */
        imu->imu_data.roll = imu->raw_packet.euler[0];
        imu->imu_data.pitch = imu->raw_packet.euler[1];
        imu->imu_data.yaw = imu->raw_packet.euler[2];
        
        /* 更新四元数数据（直接使用） */
        imu->imu_data.q[0] = imu->raw_packet.quat[0];
        imu->imu_data.q[1] = imu->raw_packet.quat[1];
        imu->imu_data.q[2] = imu->raw_packet.quat[2];
        imu->imu_data.q[3] = imu->raw_packet.quat[3];
        
        /* 更新状态 */
        imu->imu_data.update_time = HAL_GetTick();
        imu->imu_data.data_ready = 1;
        
        /* 清除新数据标志 */
        imu->new_data_flag = 0;
    }
}

/* 更新默认IMU数据（USART2） */
void HI229_UpdateData(void)
{
    hi229_update_imu_data(&hi229_usart2);
}

/* 更新USART2 IMU数据 */
void HI229_UpdateData_USART2(void)
{
    hi229_update_imu_data(&hi229_usart2);
}

/* 更新USART3 IMU数据 */
void HI229_UpdateData_USART3(void)
{
    hi229_update_imu_data(&hi229_usart3);
}

/* ====================== 数据获取函数 ====================== */

/* 获取默认IMU数据（USART2） */
hi229_imu_t* HI229_GetData(void)
{
    return &hi229_usart2.imu_data;
}

/* 获取USART2 IMU数据 */
hi229_imu_t* HI229_GetData_USART2(void)
{
    return &hi229_usart2.imu_data;
}

/* 获取USART3 IMU数据 */
hi229_imu_t* HI229_GetData_USART3(void)
{
    return &hi229_usart3.imu_data;
}

/* ====================== UART回调函数 ====================== */

/* UART接收完成回调（参考HAL_FDCAN_RxFifo0Callback风格） */
void HI229_UART_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    hi229_handle_t *imu = NULL;
    
    /* 确定是哪个IMU实例 */
    if (huart->Instance == USART2) {
        imu = &hi229_usart2;
    } else if (huart->Instance == USART3) {
        imu = &hi229_usart3;
    }
    
    if (imu == NULL || !imu->is_initialized) return;
    
    /* 保存接收长度 */
    imu->rx_len = Size;
    
    /* 处理接收到的每个字节（类似CAN数据处理） */
    for (uint16_t i = 0; i < imu->rx_len; i++) {
        hi229_process_byte(imu, imu->rx_buffer[i]);
    }
    
    /* 重新启动DMA接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, imu->rx_buffer, HI229_MAX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}