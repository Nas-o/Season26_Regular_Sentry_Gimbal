/**
 * @file bsp_flash.c
 * @brief STM32H723 内部Flash读写驱动
 * @note  使用Sector 7存储射速校准参数
 *        H723编程单位：256位(32字节)FlashWord
 *        擦除单位：整个扇区(128KB)
 */

#include "bsp_flash.h"
#include <string.h>

/**
 * @brief  从Flash读取用户数据
 * @param  data: 输出结构体指针
 * @note   如果Flash中没有有效数据(首次使用)，输出结构体清零
 */
void BSP_Flash_Read(Flash_UserData_t *data)
{
    /* 直接从Flash地址读取(Flash映射在总线上，可直接指针访问) */
    const Flash_UserData_t *flash_data = (const Flash_UserData_t *)FLASH_USER_ADDR;

    if (flash_data->valid_flag == FLASH_DATA_VALID_FLAG)
    {
        /* 有效数据，拷贝出来 */
        *data = *flash_data;
    }
    else
    {
        /* 无有效数据(首次上电/被擦除)，清零 */
        memset(data, 0, sizeof(Flash_UserData_t));
    }
}

/**
 * @brief  写入用户数据到Flash
 * @param  data: 要写入的结构体指针
 * @retval 0=成功, 1=失败
 * @note   写入流程：解锁→擦除扇区→编程→上锁
 *         注意：擦除会清除整个Sector 7(128KB)，所以所有用户参数
 *         应在一次写入中完成
 */
uint8_t BSP_Flash_Write(const Flash_UserData_t *data)
{
    HAL_StatusTypeDef status;
    uint32_t sector_error = 0;

    /* 1. 解锁Flash */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return 1;

    /* 2. 擦除Sector 7 */
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks     = FLASH_USER_BANK;
    erase_init.Sector    = FLASH_USER_SECTOR;
    erase_init.NbSectors = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    /* 3. 编程(H723每次写256位=32字节=8个uint32_t) */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                               FLASH_USER_ADDR,
                               (uint32_t)data);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    /* 4. 上锁Flash */
    HAL_FLASH_Lock();

    return 0;
}
