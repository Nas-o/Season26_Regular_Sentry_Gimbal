/*
 * @Author: Nas
 * @Date: 2026-05-24
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Algorithm\lqr_controller.c
 * @brief  LQR / LQI 状态反馈控制器实现
 */
#include "lqr_controller.hpp"
#include <string.h>

/* 内部辅助: 限幅 */
static inline float lqr_clamp(float val, float limit)
{
    if (val > limit)  return limit;
    if (val < -limit) return -limit;
    return val;
}

void LQR_Init(LQR_Controller_t *lqr,
              const float *K,
              uint8_t state_dim,
              float dt,
              float int_limit,
              float out_limit)
{
    memset(lqr, 0, sizeof(LQR_Controller_t));

    if (state_dim < 2) state_dim = 2;
    if (state_dim > LQR_STATE_DIM_MAX) state_dim = LQR_STATE_DIM_MAX;

    lqr->state_dim = state_dim;
    lqr->dt = dt;
    lqr->integral_limit = int_limit;
    lqr->output_limit = out_limit;

    for (uint8_t i = 0; i < state_dim; i++)
        lqr->K[i] = K[i];
}

float LQR_Calc(LQR_Controller_t *lqr,
               float angle_now,
               float rate_now,
               float angle_ref,
               float rate_ref,
               float u_ff)
{
    /* 状态误差 */
    float e_angle = angle_now - angle_ref;
    float e_rate  = rate_now  - rate_ref;

    lqr->state_err[0] = e_angle;
    lqr->state_err[1] = e_rate;

    /* 积分项 (LQI) */
    if (lqr->state_dim >= 3)
    {
        lqr->integral += e_angle * lqr->dt;
        lqr->integral = lqr_clamp(lqr->integral, lqr->integral_limit);
        lqr->state_err[2] = lqr->integral;
    }

    /* u = -K * state_err + u_ff */
    float u = 0.0f;
    u -= lqr->K[0] * e_angle;
    u -= lqr->K[1] * e_rate;
    if (lqr->state_dim >= 3)
        u -= lqr->K[2] * lqr->integral;

    u += u_ff;

    /* 输出限幅 */
    u = lqr_clamp(u, lqr->output_limit);

    lqr->last_output = u;
    return u;
}

void LQR_Reset(LQR_Controller_t *lqr)
{
    lqr->integral = 0.0f;
    lqr->last_output = 0.0f;
    for (uint8_t i = 0; i < LQR_STATE_DIM_MAX; i++)
        lqr->state_err[i] = 0.0f;
}

void LQR_SetGains(LQR_Controller_t *lqr, const float *K, uint8_t state_dim)
{
    if (state_dim < 2) state_dim = 2;
    if (state_dim > LQR_STATE_DIM_MAX) state_dim = LQR_STATE_DIM_MAX;

    lqr->state_dim = state_dim;
    for (uint8_t i = 0; i < state_dim; i++)
        lqr->K[i] = K[i];
}
