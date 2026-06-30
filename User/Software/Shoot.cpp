/**
 * @file   Shoot.cpp
 * @brief  发射机构控制源文件 包含 C++ 类实现：摩擦轮状态机 + 拨弹策略 + PID计算
 */
#include "Shoot.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "referee_system.hpp"
#include "User_math.hpp"
#include "Auto_control.hpp"
#include "shoot_speed_adapt.hpp"

using namespace robot::shoot;

// 实例化发射机构控制器对象
ShootController Shoot;

/*-------------------- Public: init --------------------*/

/**
 * @brief  发射机构初始化
 * @note   初始化双摩擦轮电机 (M3508), 配置速度PID参数,
 * 并从 Flash 中加载弹速自适应的初始偏移量
 */
void ShootController::init()
{
    SHOOTMotor_init(DJI_M3508, ShootMotor_L);   // 左摩擦轮电机初始化
    SHOOTMotor_init(DJI_M3508, ShootMotor_R);   // 右摩擦轮电机初始化

    // 摩擦轮速度PID配置: Kp=10, Ki=0, Kd=0, 最大输出电流=SHOOTMOTOR_MAX_CURRENT
    PID_Set(&fric_L_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    PID_Set(&fric_R_pid, 10, 0, 0, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);

    ShootSpeedAdapt_Init();  // 弹速自适应初始化 (从 Flash 读取数据)
}

/*-------------------- Private: updateState --------------------*/

/**
 * @brief  更新摩擦轮状态机
 * @note   1. 获取当前左右摩擦轮的真实转速
 * 2. 根据全局控制指令切换摩擦轮目标转速:
 * - CLOSE:       关闭摩擦轮
 * - READY:       常开摩擦轮 + 弹速闭环自适应调整
 * - DEBUG_SHOOT: 调试模式转速
 * 3. 设定死区判断，当实际转速与目标转速误差在 300 rpm 以内时，发射就绪状态置 OK
 */
void ShootController::updateState()
{
    fric_spd_L = SHOOTMotor_get_data(ShootMotor_L).speed_rpm;  // 获取左摩擦轮实际转速
    fric_spd_R = SHOOTMotor_get_data(ShootMotor_R).speed_rpm;  // 获取右摩擦轮实际转速

    static enum ShootMode s_pre_mode = CLOSE;  // 静态变量：记录上一次的射击模式（用于边缘触发）

    // 状态机切入检测：如果从 CLOSE 状态切换到其他开启状态，给摩擦轮一个初始启动速度
    if (s_pre_mode == CLOSE && Global.Shoot.shoot_mode != CLOSE) {
        fric_spd_set = SHOOT_SPEED_BEGIN;      // 设定启动初始速度 (防止瞬间电流过大)
        Global.Shoot.shoot_status = NOK;       // 发射状态：未就绪 (NOK)
        s_pre_mode = Global.Shoot.shoot_mode;
    } 
    else {
        switch(Global.Shoot.shoot_mode) {
            case CLOSE:
                fric_spd_set = SHOOT_SPEED_CLOSE; // 目标速度设为 0
                Global.Shoot.shoot_status = NOK;   // 未就绪
                break;
                
            case READY:
                // 目标速度 = 基础就绪速度 + 裁判系统/Flash 弹速自适应反馈的偏移量
                fric_spd_set = SHOOT_SPEED_READY + ShootSpeedAdapt_GetOffset();
                {
                    const float target_l = (float)SHOOT_SPEED_READY;
                    const float target_r = -(float)SHOOT_SPEED_READY; // 右电机镜像反转
                    
                    // 死区判断：左右摩擦轮实际转速与目标转速误差均小于 300 rpm 时，允许发射
                    if ((fabsf(fric_spd_L - target_l) < 300.0f) &&
                        (fabsf(fric_spd_R - target_r) < 300.0f))
                        Global.Shoot.shoot_status = OK;  // 发射状态就绪
                    else
                        Global.Shoot.shoot_status = NOK; // 摩擦轮还在提速或不稳定
                }
                break;
                
            case DEBUG_SHOOT:
                fric_spd_set = SHOOT_SPEED_BEGIN;
                Global.Shoot.shoot_status = OK;   // 调试模式直接强置就绪
                break;
                
            default:
                fric_spd_set = SHOOT_SPEED_CLOSE;
                Global.Shoot.shoot_status = NOK;
                break;
        }
        s_pre_mode = Global.Shoot.shoot_mode; // 更新历史状态
    }
}

/*-------------------- Private: selectTriggerMode --------------------*/

/**
 * @brief  选择拨弹轴（触发器）的工作模式
 * @note   优先级策略：
 * 1. 当底盘/云台锁死 (LOCK) 或摩擦轮关闭时 -> 强制关闭拨弹
 * 2. 自动控制模式（自瞄/打符）下 -> 由视觉指令和单片机（MCU）发射许可共同决定
 * 3. 手动遥控模式下 -> 只有摩擦轮转速达到 OK 状态时才允许响应手动拨弹指令
 */
void ShootController::selectTriggerMode()
{
    // 安全保护：若系统处于锁死状态或摩擦轮未开启，关闭拨弹
    if (Global.Control.mode == LOCK || Global.Shoot.shoot_mode == CLOSE)
    {
        Global.Shoot.trigger_mode = TRIGGER_CLOSE;
    }
    // 自动模式：自瞄或能量机关（打符）状态
    else if (Global.Auto.mode != NONE)
    {
        if (!Global.Shoot.mcu_fire_permit)          // 检查单片机自身物理限频/热量保护许可
            Global.Shoot.trigger_mode = TRIGGER_CLOSE;
        else if (toMINIPC.mode == MODE_SMALL_BUFF || toMINIPC.mode == MODE_BIG_BUFF)
            Global.Shoot.trigger_mode = SINGLE;     // 能量机关模式：采用单发（SINGLE）策略精准击打
        else
            Global.Shoot.trigger_mode = ONLY_AIM_HIGH; // 常规自瞄模式：采用高频连发或特定自瞄发射策略
    }
    // 手动模式：无视觉目标
    else
    {
        // 只有当摩擦轮转速 OK（稳定）时，才采纳操控手的手动拨弹指令，否则关闭拨弹防止卡弹或滑弹
        Global.Shoot.trigger_mode = (Global.Shoot.shoot_status == OK)
            ? Global.Shoot.manual_trigger_mode : TRIGGER_CLOSE;
    }
}

/*-------------------- Private: calculate --------------------*/

/**
 * @brief  摩擦轮速度闭环 PID 计算
 * @note   输入当前速度与目标速度，输出控制电流（右电机取反实现反向对转）
 */
void ShootController::calculate()
{
    current[0] = PID_Cal(&fric_L_pid, fric_spd_L,  fric_spd_set);  // 左摩擦轮正转 PID
    current[1] = PID_Cal(&fric_R_pid, fric_spd_R, -fric_spd_set);  // 右摩擦轮反转 PID
}

/*-------------------- Private: output --------------------*/

/**
 * @brief  输出控制电流到电机总线
 * @note   将 PID 计算出的电流值通过 CAN 总线发送给 M3508 电调
 */
void ShootController::output()
{
    SHOOTMotor_set(current[0], ShootMotor_L);
    SHOOTMotor_set(current[1], ShootMotor_R);
}

/*-------------------- Public: task --------------------*/

/**
 * @brief  发射机构控制主任务 (运行频率通常为 1kHz)
 * @note   控制时序：更新状态机 -> 选择拨弹模式 -> PID计算 -> 电流输出
 * 可以通过宏定义 USE_SHOOT 整体关闭整个发射机构的硬件控制
 */
void ShootController::task()
{
#if (USE_SHOOT != 0)
    updateState();        // 1. 更新摩擦轮实际速度，判定就绪状态
    selectTriggerMode();  // 2. 根据当前云台/视觉状态选择拨弹轴模式
    calculate();          // 3. 计算摩擦轮 PID 速度闭环
    output();             // 4. 发送电流数据驱动电机
#endif
}

/* ============================================================
 * C 风格接口封装 (用于 FreeRTOS 任务控制块 / C 核心回调)
 * ============================================================ */

void Shoot_Init()  { Shoot.init(); }
void Shoot_Tasks() { Shoot.task(); }