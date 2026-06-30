/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:56:49
 * @FilePath: \Season-26-Code\User\Software\music.c
 */
/**
 * @file music.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief 音乐播放模块
 * @version 0.1
 * @date 2025-04-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "music.hpp"

// ===== 《两只老虎》旋律数组 =====
Note melody[] = {
    // 第一段（中音区）
    {kMidDo, 500},
    {kMidRe, 500},
    {kMidMi, 500},
    {kMidDo, 500}, // 1 2 3 1
    {kMidDo, 500},
    {kMidRe, 500},
    {kMidMi, 500},
    {kMidDo, 500}, // 1 2 3 1
    {kMidMi, 500},
    {kMidFa, 500},
    {kMidSol, 1000}, // 3 4 5 -
    {kMidMi, 500},
    {kMidFa, 500},
    {kMidSol, 1000}, // 3 4 5 -

    // 第二段（中音区 + 高音区）
    {kMidSol, 250},
    {kMidLa, 250},
    {kMidSol, 250},
    {kMidFa, 250},
    {kMidMi, 500},
    {kMidDo, 500}, // 5 6 5 4 3 1
    {kMidSol, 250},
    {kMidLa, 250},
    {kMidSol, 250},
    {kMidFa, 250},
    {kMidMi, 500},
    {kMidDo, 500}, // 5 6 5 4 3 1

    // 结尾部分（中音Do + 高音Sol）
    {kMidDo, 500},
    {kHighSol, 500},
    {kMidDo, 1000}, // 1 5 1 -
    {kMidDo, 500},
    {kHighSol, 500},
    {kMidDo, 1000}, // 1 5 1 -
    {kMusicFinish, 500},
};

/**
 * @brief  蜂鸣器初始化
 * @note   调用底层BSP蜂鸣器初始化
 */
void Music_Init()
{
    Buzzer_Init();
}

/**
 * @brief  播放乐谱
 * @param  note 音符数组指针, 以 kMusicFinish(频率=0) 结尾
 * @note   阻塞式播放: 逐个音符设置频率并延时, 直到遇到结束标志
 */
void Music_Play(Note *note)
{
    for (int i = 0; note[i].fre != kMusicFinish; i++)
    {
        MUSIC_SET_FREQUENCY(note[i].fre);  // 设置PWM频率
        MUSIC_ON();                         // 开启蜂鸣器
        MUSIC_DELAY(note[i].time);         // 延时 (使用RTOS延时, 不阻塞其他任务)
    }
    MUSIC_OFF();  // 播放完毕: 关闭蜂鸣器
}
