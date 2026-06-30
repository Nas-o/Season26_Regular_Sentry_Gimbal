/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-05 07:43:44
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Software\music.hpp
 */
#ifndef __MUSIC_H__

#ifdef __cplusplus
extern "C" 
{
#endif

    #define __MUSIC_H__
    #include "buzzer.hpp"
    #include "cmsis_os2.h"

    typedef struct
    {
        float fre;
        uint16_t time;
    } Note;



    #define MUSIC_SET_FREQUENCY(fre) Buzzer_SetFrequency(fre)
    #define MUSIC_ON() Buzzer_On()
    #define MUSIC_OFF() Buzzer_Off()
    #define MUSIC_DELAY(time) osDelay(time)

#ifdef __cplusplus
} // extern "C"

// ===== C大调低音区（C3-B3）=====
constexpr float kLowDo  = 130.81f;  // C3
constexpr float kLowRe  = 146.83f;  // D3
constexpr float kLowMi  = 164.81f;  // E3
constexpr float kLowFa  = 174.61f;  // F3
constexpr float kLowSol = 196.00f;  // G3
constexpr float kLowLa  = 220.00f;  // A3
constexpr float kLowSi  = 246.94f;  // B3

// ===== C大调中音区（C4-B4）=====
constexpr float kMidDo  = 261.63f;  // C4 (中央C)
constexpr float kMidRe  = 293.66f;  // D4
constexpr float kMidMi  = 329.63f;  // E4
constexpr float kMidFa  = 349.23f;  // F4
constexpr float kMidSol = 392.00f;  // G4
constexpr float kMidLa  = 440.00f;  // A4 (标准音)
constexpr float kMidSi  = 493.88f;  // B4

// ===== C大调高音区（C5-B5）=====
constexpr float kHighDo  = 523.25f;  // C5
constexpr float kHighRe  = 587.33f;  // D5
constexpr float kHighMi  = 659.26f;  // E5
constexpr float kHighFa  = 698.46f;  // F5
constexpr float kHighSol = 783.99f;  // G5
constexpr float kHighLa  = 880.00f;  // A5
constexpr float kHighSi  = 987.77f;  // B5

// ===== C大调结束标志=====
constexpr float kMusicFinish = 0.00f; // 结束

/* ---- 向后兼容旧宏名 (供旧代码引用) ---- */
#define LOW_DO    kLowDo
#define LOW_RE    kLowRe
#define LOW_MI    kLowMi
#define LOW_FA    kLowFa
#define LOW_SOL   kLowSol
#define LOW_LA    kLowLa
#define LOW_SI    kLowSi
#define MID_DO    kMidDo
#define MID_RE    kMidRe
#define MID_MI    kMidMi
#define MID_FA    kMidFa
#define MID_SOL   kMidSol
#define MID_LA    kMidLa
#define MID_SI    kMidSi
#define HIGH_DO   kHighDo
#define HIGH_RE   kHighRe
#define HIGH_MI   kHighMi
#define HIGH_FA   kHighFa
#define HIGH_SOL  kHighSol
#define HIGH_LA   kHighLa
#define HIGH_SI   kHighSi
#define MUSIC_FINISH kMusicFinish

extern "C" {
#else
// ===== C大调低音区（C3-B3）=====
#define LOW_DO 130.81f  // C3
#define LOW_RE 146.83f  // D3
#define LOW_MI 164.81f  // E3
#define LOW_FA 174.61f  // F3
#define LOW_SOL 196.00f // G3
#define LOW_LA 220.00f  // A3
#define LOW_SI 246.94f  // B3

// ===== C大调中音区（C4-B4）=====
#define MID_DO 261.63f  // C4 (中央C)
#define MID_RE 293.66f  // D4
#define MID_MI 329.63f  // E4
#define MID_FA 349.23f  // F4
#define MID_SOL 392.00f // G4
#define MID_LA 440.00f  // A4 (标准音)
#define MID_SI 493.88f  // B4

// ===== C大调高音区（C5-B5）=====
#define HIGH_DO 523.25f  // C5
#define HIGH_RE 587.33f  // D5
#define HIGH_MI 659.26f  // E5
#define HIGH_FA 698.46f  // F5
#define HIGH_SOL 783.99f // G5
#define HIGH_LA 880.00f  // A5
#define HIGH_SI 987.77f  // B5

// ===== C大调结束标志=====
#define MUSIC_FINISH 0.00f // 结束
#endif

void Music_Init();
void Music_Play(Note *note);

extern Note melody[];


#ifdef __cplusplus
}
#endif
#endif