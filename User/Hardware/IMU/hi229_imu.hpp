/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-12-08 19:24:42
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2025-12-12 04:09:25
 * @FilePath: \Season-26-Code-body\User\Hardware\IMU\HI229_IMU.h
 */

#ifndef __HI229_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __HI229_IMU_H__

#include "main.h"
#include "usart.h"

/* HI229 IMU模块配置 */
#define HI229_USE_RS485            0       /* 0:使用普通串口 1:使用RS485 */
#define HI229_MAX_BUFFER_SIZE      256     /* 接收缓冲区大小 */
#define HI229_MAX_FRAME_SIZE       (HI229_MAX_BUFFER_SIZE + 6)

/* HI229协议常量定义 */
#define HI229_SYNC1                0x5A    /* 帧头 */
#define HI229_SYNC2                0xA5    /* 帧类型 */
#define HI229_HEADER_SIZE          6       /* 帧头大小 */
#define HI229_TAG_IMUSOL           0x91    /* IMUSOL数据包 */

/* 陀螺仪数据结构体（参考dm_imu.h风格） */
typedef struct {
    float gyro[3];                  /* 角速度 (rad/s) [x, y, z] */
    float accel[3];                 /* 加速度 (m/s?) [x, y, z] */
    float roll;                     /* 横滚角 (deg) */
    float pitch;                    /* 俯仰角 (deg) */
    float yaw;                      /* 航向角 (deg) */
    float q[4];                     /* 四元数 (w, x, y, z) */
    uint32_t update_time;           /* 最后更新时间戳 */
    uint8_t data_ready;             /* 新数据就绪标志 */
} hi229_imu_t;

/* IMUSOL数据包结构体 */
#pragma pack(push, 1)
typedef struct {
    uint8_t tag;                    /* 数据包标签: 0x91 */
    uint8_t id;                     /* 模块ID */
    uint8_t rev[2];                 /* 保留字节 */
    float pressure;                 /* 气压 (Pa) */
    uint32_t system_time;           /* 时间戳 (ms) */
    float acc[3];                   /* 加速度 (G) */
    float gyro[3];                  /* 角速度 (deg/s) */
    float mag[3];                   /* 磁场强度 (uT) */
    float euler[3];                 /* 欧拉角 (roll, pitch, yaw) (deg) */
    float quat[4];                  /* 四元数 (w, x, y, z) */
} hi229_imusol_t;
#pragma pack(pop)

/* IMU设备结构体（参考dm_imu.c风格） */
typedef struct {
    /* 硬件接口 */
    UART_HandleTypeDef *huart;      /* UART句柄 */
    DMA_HandleTypeDef *hdma_rx;     /* DMA接收句柄 */
    
    /* 数据缓冲区 */
    uint8_t rx_buffer[HI229_MAX_BUFFER_SIZE];   /* 接收缓冲区 */
    uint16_t rx_len;                /* 接收数据长度 */
    
    /* 数据解析 */
    uint8_t raw_buffer[HI229_MAX_FRAME_SIZE];   /* 原始数据缓冲区 */
    uint16_t raw_index;             /* 缓冲区索引 */
    uint16_t frame_len;             /* 帧长度 */
    
    /* IMU数据 */
    hi229_imu_t imu_data;           /* IMU数据 */
    hi229_imusol_t raw_packet;      /* 原始数据包 */
    
    /* 状态 */
    uint8_t is_initialized;         /* 初始化标志 */
    uint8_t parse_state;            /* 解析状态 */
    uint8_t new_data_flag;          /* 新数据标志（时钟中断用） */
} hi229_handle_t;

/* 全局IMU实例（参考dm_imu.c） */
extern hi229_handle_t hi229_usart2;
extern hi229_handle_t hi229_usart3;

/* ====================== 初始化函数 ====================== */
void HI229_Init(hi229_handle_t *imu, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);
void HI229_Start(hi229_handle_t *imu);

/* ====================== 数据获取函数 ====================== */
hi229_imu_t* HI229_GetData(void);  /* 获取默认IMU数据（USART2） */
hi229_imu_t* HI229_GetData_USART2(void);  /* 获取USART2 IMU数据 */
hi229_imu_t* HI229_GetData_USART3(void);  /* 获取USART3 IMU数据 */

/* ====================== 时钟中断调用函数 ====================== */
/* 在时钟中断中调用，处理新数据 */
void HI229_UpdateData(void);  /* 更新默认IMU数据 */
void HI229_UpdateData_USART2(void);  /* 更新USART2 IMU数据 */
void HI229_UpdateData_USART3(void);  /* 更新USART3 IMU数据 */

/* ====================== UART回调函数 ====================== */
void HI229_UART_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);


#ifdef __cplusplus
}
#endif
#endif /* __HI229_IMU_H__ */