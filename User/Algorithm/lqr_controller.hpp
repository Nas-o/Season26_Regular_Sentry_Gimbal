/*
 * @Author: Nas
 * @Date: 2026-05-24
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Algorithm\lqr_controller.h
 * @brief  LQR / LQI 状态反馈控制器
 *         u = -K * (x - x_ref) + u_ff
 *         支持 2 态 (角度+角速度) 或 3 态 (+ 积分) 的 LQI
 */
#ifndef __LQR_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __LQR_CONTROLLER_H__

#include "struct_typedef.hpp"

/* 最大状态维度 (角度, 角速度, 积分误差) */
#define LQR_STATE_DIM_MAX  3

typedef struct
{
    /* --- 离线计算好的增益 (由 MATLAB dlqr 得到) --- */
    float K[LQR_STATE_DIM_MAX];   // 反馈增益向量 [K_angle, K_rate, K_integral]
    uint8_t state_dim;            // 实际使用的状态维度 (2=LQR, 3=LQI)

    /* --- 运行时状态 --- */
    float integral;               // 角度误差积分 (仅 state_dim==3 时使用)
    float integral_limit;         // 积分限幅
    float dt;                     // 控制周期 (s)

    /* --- 输出限幅 --- */
    float output_limit;           // 控制量绝对值限幅

    /* --- 调试用 --- */
    float last_output;            // 上一拍输出
    float state_err[LQR_STATE_DIM_MAX]; // 上一拍状态误差 (方便 vofa 观测)

} LQR_Controller_t;

/**
 * @brief 初始化 LQR/LQI 控制器
 * @param lqr        控制器实例
 * @param K          增益数组 (长度 = state_dim)
 * @param state_dim  状态维度 2 或 3
 * @param dt         控制周期 (s), 通常 0.001
 * @param int_limit  积分限幅 (仅 LQI 有效)
 * @param out_limit  输出限幅
 */
void LQR_Init(LQR_Controller_t *lqr,
              const float *K,
              uint8_t state_dim,
              float dt,
              float int_limit,
              float out_limit);

/**
 * @brief 计算一拍 LQR/LQI 控制输出
 * @param lqr        控制器实例
 * @param angle_now  当前角度 (deg)
 * @param rate_now   当前角速度 (deg/s)
 * @param angle_ref  目标角度 (deg)
 * @param rate_ref   目标角速度 (deg/s), 通常=0 或前馈速度
 * @param u_ff       前馈控制量 (如重力补偿电流)
 * @return           控制输出 (电流/力矩指令)
 */
float LQR_Calc(LQR_Controller_t *lqr,
               float angle_now,
               float rate_now,
               float angle_ref,
               float rate_ref,
               float u_ff);

/**
 * @brief 清零积分及内部状态 (模式切换时调用)
 */
void LQR_Reset(LQR_Controller_t *lqr);

/**
 * @brief 在线修改增益 (方便串口/vofa 调参)
 */
void LQR_SetGains(LQR_Controller_t *lqr, const float *K, uint8_t state_dim);


#ifdef __cplusplus
}
#endif
#endif /* __LQR_CONTROLLER_H__ */
