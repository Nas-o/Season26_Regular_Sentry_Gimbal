/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas-o 1319621819@qq.com
 * @LastEditTime: 2026-07-27 09:25:20
 * @FilePath: \Reserve_Sentry\User\Algorithm\pid.c
 */
#include "math.h"
#include "pid.hpp"

void PID_Set(pid_t *PidSet,float p_set,float i_set,float d_set,float f_set,float lim_out_set,float lim_i_outset)//PID设置
{
  PidSet->p = p_set;   PidSet->i = i_set;   PidSet->d = d_set; PidSet->f = f_set;

  PidSet->lim_out = lim_out_set;   PidSet->lim_i_out = lim_i_outset;//将设置赋值
  PID_Reset(PidSet, 0.0f, 0.0f);
}

//PID计算
// PID计算
float PID_Cal(pid_t *PidGoal, float Now, float Set)
{
    // 1. 数据合法性保护：异常时仅清零输出，切记不要重置历史 err，防止微分冲击！
    if (!isfinite(Now) || !isfinite(Set) || !isfinite(PidGoal->i_out))
    {
        PidGoal->p_out = 0.0f;
        PidGoal->i_out = 0.0f;
        PidGoal->d_out = 0.0f;
        PidGoal->total_out = 0.0f;
        return 0.0f;
    }

    // 2. 刷新误差与微分 (逻辑与稳定版完全一致)
    PidGoal->set = Set;
    PidGoal->err_last = PidGoal->err;
    PidGoal->err = Set - Now;
    PidGoal->diff = PidGoal->err - PidGoal->err_last;

    // 3. 基础 P, I, D, F 各项独立计算
    PidGoal->p_out = PidGoal->p * PidGoal->err;
    
    if (PidGoal->i != 0.0f)
        PidGoal->i_out += PidGoal->i * PidGoal->err;
        
    PidGoal->d_out = PidGoal->d * PidGoal->diff;
    PidGoal->f_out = PidGoal->f * Set;

    // 4. 积分独立限幅 (稳定版的经典钳位)
    if (PidGoal->i_out > PidGoal->lim_i_out)
        PidGoal->i_out = PidGoal->lim_i_out;
    else if (PidGoal->i_out < -PidGoal->lim_i_out)
        PidGoal->i_out = -PidGoal->lim_i_out;

    // 5. 总输出计算与限幅
    PidGoal->total_out = PidGoal->p_out + PidGoal->i_out + PidGoal->d_out + PidGoal->f_out;

    if (PidGoal->total_out > PidGoal->lim_out)
        PidGoal->total_out = PidGoal->lim_out;
    else if (PidGoal->total_out < -PidGoal->lim_out)
        PidGoal->total_out = -PidGoal->lim_out;

    return PidGoal->total_out;
}

void PID_Reset(pid_t *pid, float now, float set)
{
    pid->set = set;
    pid->err = set - now;
    pid->err_last = pid->err;   // 避免首拍D冲击
    pid->diff = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->f_out = 0.0f;
    pid->total_out = 0.0f;
}
