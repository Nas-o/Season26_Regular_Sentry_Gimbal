/**
 * @file bsp_flash.h
 * @brief STM32H723 内部Flash读写驱动（用于射速校准值持久化存储）
 */

#ifndef __BSP_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __BSP_FLASH_H__

#include "main.h"

/* STM32H723 Flash Sector 7 (最后一个扇区，用于存储用户参数) */
#define FLASH_USER_SECTOR       FLASH_SECTOR_7
#define FLASH_USER_ADDR         ((uint32_t)0x080E0000)  // Sector 7 起始地址
#define FLASH_USER_BANK         FLASH_BANK_1

/* 数据有效标志（防止首次上电读到全0xFF的脏数据） */
#define FLASH_DATA_VALID_FLAG   ((uint32_t)0x26AD0026)

/* 存储结构体（必须32字节对齐，因为H723编程单位为256位=32字节） */
typedef struct __attribute__((aligned(32)))
{
    uint32_t valid_flag;          // 数据有效标志
    int16_t  shoot_speed_offset;  // 摩擦轮转速偏移量 (RPM)
    uint16_t reserved;            // 保留对齐
    uint8_t  padding[24];         // 填充到32字节
} Flash_UserData_t;

void    BSP_Flash_Read(Flash_UserData_t *data);
uint8_t BSP_Flash_Write(const Flash_UserData_t *data);


#ifdef __cplusplus
}
#endif
#endif /* __BSP_FLASH_H__ */
