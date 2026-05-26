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
    {MID_DO, 500},
    {MID_RE, 500},
    {MID_MI, 500},
    {MID_DO, 500}, // 1 2 3 1
    {MID_DO, 500},
    {MID_RE, 500},
    {MID_MI, 500},
    {MID_DO, 500}, // 1 2 3 1
    {MID_MI, 500},
    {MID_FA, 500},
    {MID_SOL, 1000}, // 3 4 5 -
    {MID_MI, 500},
    {MID_FA, 500},
    {MID_SOL, 1000}, // 3 4 5 -

    // 第二段（中音区 + 高音区）
    {MID_SOL, 250},
    {MID_LA, 250},
    {MID_SOL, 250},
    {MID_FA, 250},
    {MID_MI, 500},
    {MID_DO, 500}, // 5 6 5 4 3 1
    {MID_SOL, 250},
    {MID_LA, 250},
    {MID_SOL, 250},
    {MID_FA, 250},
    {MID_MI, 500},
    {MID_DO, 500}, // 5 6 5 4 3 1

    // 结尾部分（中音Do + 高音Sol）
    {MID_DO, 500},
    {HIGH_SOL, 500},
    {MID_DO, 1000}, // 1 5 1 -
    {MID_DO, 500},
    {HIGH_SOL, 500},
    {MID_DO, 1000}, // 1 5 1 -
    {MUSIC_FINISH,500},
};

void Music_Init()
{
    Buzzer_Init();
}

void Music_Play(Note *note)
{
    for (int i = 0; note[i].fre!=MUSIC_FINISH; i++)
    {
        MUSIC_SET_FREQUENCY(note[i].fre);
        MUSIC_ON();
        MUSIC_DELAY(note[i].time);
    }
    MUSIC_OFF();
}
