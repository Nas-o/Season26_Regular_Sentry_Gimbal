/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas-o 1319621819@qq.com
 * @LastEditTime: 2026-07-17 23:43:47
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
float PID_Cal(pid_t *PidGoal, float Now, float Set) // PID
{
  if (!isfinite(Now) || !isfinite(Set) ||
      !isfinite(PidGoal->i_out) ||
      PidGoal->lim_out <= 0.0f || PidGoal->lim_i_out < 0.0f)
  {
    PID_Reset(PidGoal, 0.0f, 0.0f);
    return 0.0f;
  }

  PidGoal->set = Set;
  PidGoal->err_last = PidGoal->err;
  PidGoal->err = Set - Now; // 计算误差
  PidGoal->diff = PidGoal->err - PidGoal->err_last;
   
  PidGoal->p_out = PidGoal->p * PidGoal->err;
  
  // 条件积分：输出饱和时只允许积分向退出饱和的方向变化
  if(PidGoal->i != 0)
  {
    float integral_delta = PidGoal->i *
                           (PidGoal->err + PidGoal->err_last) * 0.5f;
    float candidate_i = PidGoal->i_out + integral_delta;

    if (candidate_i > PidGoal->lim_i_out)
      candidate_i = PidGoal->lim_i_out;
    else if (candidate_i < -PidGoal->lim_i_out)
      candidate_i = -PidGoal->lim_i_out;

    float candidate_total = PidGoal->p_out + candidate_i
                          + PidGoal->d * PidGoal->diff
                          + PidGoal->f * Set;
    bool pushes_high = candidate_total > PidGoal->lim_out && integral_delta > 0.0f;
    bool pushes_low  = candidate_total < -PidGoal->lim_out && integral_delta < 0.0f;

    if (!pushes_high && !pushes_low)
      PidGoal->i_out = candidate_i;
  }
    
  PidGoal->d_out = PidGoal->d * PidGoal->diff; // pid运算
  PidGoal->f_out = PidGoal->f * Set; // 前馈运算
  
  if(fabs(PidGoal->i_out) > PidGoal->lim_i_out) // 防止积分过大
  {
    if(PidGoal->i_out < 0)
      PidGoal->i_out = -PidGoal->lim_i_out;
    else
      PidGoal->i_out = PidGoal->lim_i_out;
  }
  
  PidGoal->total_out = PidGoal->p_out + PidGoal->i_out + PidGoal->d_out + PidGoal->f_out; // 计算总和输出  

  if(fabs(PidGoal->total_out) > PidGoal->lim_out) // 防止总和过大
  {
    if(PidGoal->total_out < 0)
      PidGoal->total_out = -PidGoal->lim_out;
    else
      PidGoal->total_out = PidGoal->lim_out;
  }
  
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
