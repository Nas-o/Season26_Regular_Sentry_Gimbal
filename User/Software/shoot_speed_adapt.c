/**
 * @file  shoot_speed_adapt.c
 * @brief 射速动态自适应模块
 * @note  采集近10发弹速均值，动态微调摩擦轮转速偏移量并持久化到Flash
 */

#include "shoot_speed_adapt.h"
#include "bsp_flash.h"
#include "referee_system.h"

static float   speed_queue[SPEED_QUEUE_SIZE]; // 环形队列
static uint8_t queue_idx   = 0;              // 写入位置
static uint8_t queue_count = 0;              // 已有数据量
static float   last_speed  = 0;              // 上一次记录的弹速
static int16_t speed_offset = 0;             // 当前摩擦轮转速偏移量 (RPM)

/**
 * @brief  初始化：从Flash恢复上次的偏移量
 */
void ShootSpeedAdapt_Init(void)
{
    Flash_UserData_t flash_data;
    BSP_Flash_Read(&flash_data);

    if (flash_data.valid_flag == FLASH_DATA_VALID_FLAG)
        speed_offset = flash_data.shoot_speed_offset;
    else
        speed_offset = 0;

    queue_idx   = 0;
    queue_count = 0;
    last_speed  = 0;
}

/**
 * @brief  将偏移量写入Flash
 */
static void save_offset_to_flash(void)
{
    Flash_UserData_t data = {0};
    data.valid_flag = FLASH_DATA_VALID_FLAG;
    data.shoot_speed_offset = speed_offset;
    BSP_Flash_Write(&data);
}

/**
 * @brief  周期调用（建议在发射任务中调用）
 * @note   自动检测新弹丸射速，填入队列，满10发后计算均值并判断是否调整
 */
void ShootSpeedAdapt_Update(void)
{
    float cur_speed = Referee_data.Initial_SPEED;

    /* 1. 检测是否为新弹丸：弹速有效且与上次不同 */
    if (cur_speed < 10.0f || cur_speed > 35.0f)
        return; // 无效数据，忽略
    if (cur_speed == last_speed)
        return; // 未发射新弹

    last_speed = cur_speed;

    /* 2. 压入环形队列 */
    speed_queue[queue_idx] = cur_speed;
    queue_idx = (queue_idx + 1) % SPEED_QUEUE_SIZE;
    if (queue_count < SPEED_QUEUE_SIZE)
        queue_count++;

    /* 3. 队列未满，不做判断 */
    if (queue_count < SPEED_QUEUE_SIZE)
        return;

    /* 4. 计算10发均值 */
    float sum = 0;
    for (uint8_t i = 0; i < SPEED_QUEUE_SIZE; i++)
        sum += speed_queue[i];
    float avg = sum / (float)SPEED_QUEUE_SIZE;

    /* 5. 阈值判断并调整 */
    if (avg > SPEED_HIGH_THRESH)
    {
        speed_offset -= SPEED_ADJUST_STEP;
        if (speed_offset < SPEED_OFFSET_MIN)
            speed_offset = SPEED_OFFSET_MIN;
        save_offset_to_flash();
    }
    else if (avg < SPEED_LOW_THRESH)
    {
        speed_offset += SPEED_ADJUST_STEP;
        if (speed_offset > SPEED_OFFSET_MAX)
            speed_offset = SPEED_OFFSET_MAX;
        save_offset_to_flash();
    }
    /* 23~24 m/s 之间不调整 */
}

/**
 * @brief  获取当前摩擦轮转速偏移量
 * @return 偏移量 (RPM)，正值=提速，负值=降速
 */
int16_t ShootSpeedAdapt_GetOffset(void)
{
    return speed_offset;
}
