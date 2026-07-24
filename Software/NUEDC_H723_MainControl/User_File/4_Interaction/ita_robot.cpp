/**
 * @file ita_robot.cpp
 * @author WangFangzhuo
 * @brief 整车人机交互与操作逻辑
 * @version 1.0
 * @date 2026-07-18
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_robot.h"
#include "alg_basic.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// 整车人机交互控制类
Class_Robot Robot;

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化整车操作逻辑
 *
 * @param __DR16 DR16遥控器对象
 * @param __Chassis 两轮差速底盘对象
 */
void Class_Robot::Init(Class_DR16 *__DR16, Class_Chassis *__Chassis)
{
    DR16 = __DR16;
    Chassis = __Chassis;

    Chassis_Control_Enable = false;

    if (Chassis != nullptr)
    {
        Chassis->Set_Control_State(Chassis_Control_State_DISABLE);
    }
}

/**
 * @brief 对摇杆输入应用死区并重新映射到[-1, 1]
 *
 * @note 不是简单地把死区内清零，而是把死区外剩余行程重新拉伸，
 *       保证摇杆推到机械极限时输出仍可达到正负1。
 */
float Class_Robot::Apply_Dead_Zone(float Input) const
{
    Input = Basic_Math_Constrain(Input, -1.0f, 1.0f);

    const float input_abs = Basic_Math_Abs(Input);

    if (input_abs <= DR16_Dead_Zone)
    {
        return (0.0f);
    }

    const float output_abs = (input_abs - DR16_Dead_Zone) / (1.0f - DR16_Dead_Zone);

    return ((Input >= 0.0f) ? output_abs : -output_abs);
}

/**
 * @brief 安全失能底盘
 */
void Class_Robot::Disable_Chassis()
{
    Chassis_Control_Enable = false;

    if (Chassis == nullptr)
    {
        return;
    }

    /*
     * Set_Control_State(DISABLE)内部会清空：
     * 1. 底盘Vx和Omega目标；
     * 2. 左右轮目标角速度；
     * 3. 双路电机控制器。
     *
     * 仅在状态发生变化时调用，避免每1ms重复清空控制器。
     */
    if (Chassis->Get_Control_State() != Chassis_Control_State_DISABLE)
    {
        Chassis->Set_Control_State(Chassis_Control_State_DISABLE);
    }
}

/**
 * @brief 1ms整车操作逻辑回调函数
 */
void Class_Robot::TIM_1ms_Control_PeriodElapsedCallback()
{
    if (DR16 == nullptr || Chassis == nullptr)
    {
        return;
    }

    /*
     * 底盘使能条件：
     * 1. DR16必须在线；
     * 2. 右拨杆必须稳定处于UP。
     *
     * MIDDLE、DOWN和所有拨杆过渡状态均按失能处理，
     * 避免拨杆切换过程中出现非预期运动。
     */
    const bool dr16_online = (DR16->Get_DR16_Status() == DR16_Status_ENABLE);

    const bool chassis_enable_request = (DR16->Get_Right_Switch() == DR16_Switch_Status_UP);

    if (!dr16_online || !chassis_enable_request)
    {
        Disable_Chassis();
        return;
    }

    if (Chassis->Get_Control_State() != Chassis_Control_State_NORMAL)
    {
        Chassis->Set_Control_State(Chassis_Control_State_NORMAL);
    }

    Chassis_Control_Enable = true;

    // 左摇杆Y轴控制底盘前进/后退
    const float target_velocity_x = Apply_Dead_Zone(DR16->Get_Left_Y()) * Chassis_Max_Velocity_X * ROBOT_CHASSIS_VELOCITY_X_SIGN;

    // 左摇杆X轴控制底盘旋转
    const float target_omega = Apply_Dead_Zone(DR16->Get_Left_X()) * Chassis_Max_Omega * ROBOT_CHASSIS_OMEGA_SIGN;

    Chassis->Set_Target_Velocity_X(target_velocity_x);
    Chassis->Set_Target_Omega(target_omega);
}

/*----------------------------------------------------------------------------*/
