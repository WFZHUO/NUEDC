/**
 * @file crt_chassis.cpp
 * @author WangFonzhuo
 * @brief 两轮差速底盘运动解算
 * @version 1.0
 * @date 2026-07-18
 * @note TI驱动板通道映射：Motor_A为右轮，Motor_B为左轮。
 * @note 单轮机械角速度以从输出轴侧向电机内部看逆时针为正。
 *       当前TI反馈已经使用该正方向；电机Direction只修正PWM输出极性，
 *       不改变反馈角度与角速度的正负方向。
 * @note 左右电机镜像安装。左轮机械正方向对应车体向前，右轮机械负方向
 *       对应车体向前。底盘层通过CHASSIS_xxx_WHEEL_FORWARD_SIGN将电机
 *       机械角速度转换为“驱动车体向前为正”的车轮逻辑角速度。
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化底盘
 *
 * @param huart 与TI驱动板连接的UART
 * @param __Wheel_Radius 轮子半径，单位m
 * @param __Chassis_Half_Track 左右驱动轮轮心横向距离的一半，单位m
 * @param __Wheel_Linear_Speed_Limit 单轮线速度限幅，单位m/s，0表示不限制
 * @param __Encoder_Num_Per_Round 输出轴每圈软件累计计数
 * @param __Direction_Left 左轮PWM输出方向
 * @param __Direction_Right 右轮PWM输出方向
 *
 * @note 当前实车左右轮均使用Motor_TI_Direction_REVERSE。
 * @note Class_Motor_TI::Init的方向参数顺序为A、B；本底盘A路为右轮、
 *       B路为左轮，因此这里按右轮、左轮顺序传入底层电机对象。
 */
void Class_Chassis::Init(
    UART_HandleTypeDef *huart,
    float __Wheel_Radius,
    float __Chassis_Half_Track,
    float __Wheel_Linear_Speed_Limit,
    float __Encoder_Num_Per_Round,
    Enum_Motor_TI_Direction __Direction_Left,
    Enum_Motor_TI_Direction __Direction_Right)
{
    if (__Wheel_Radius > 0.0f)
    {
        Wheel_Radius = __Wheel_Radius;
    }

    if (__Chassis_Half_Track > 0.0f)
    {
        Chassis_Half_Track = __Chassis_Half_Track;
    }

    Set_Wheel_Linear_Speed_Limit(
        __Wheel_Linear_Speed_Limit);

    /*
     * TI驱动板通道映射：
     * Motor_A为右轮，使用右轮方向参数；
     * Motor_B为左轮，使用左轮方向参数。
     *
     * Direction只修正PWM输出极性，不改变TI上传的编码器反馈方向。
     * 当前实车两路默认均为REVERSE，使电机机械正目标角速度对应
     * 输出轴侧向内看逆时针旋转，并与机械正反馈角速度一致。
     * 镜像安装导致的左右轮前进方向差异由底盘层符号映射处理，
     * 不应通过修改某一路Direction破坏电机自身速度闭环方向。
     */
    Motor_TI.Init(huart,
                  __Encoder_Num_Per_Round,
                  __Direction_Right,
                  __Direction_Left);

    // 右轮A路使用角速度闭环，角速度环输出单位为PWM
    Motor_TI.Motor_A.Set_Control_Method(
        Motor_TI_Control_Method_OMEGA);
    Motor_TI.Motor_A.PID_Omega.Init(
        125.0f, 1500.0f, 0.0f);
    Motor_TI.Motor_A.PID_Angle.Init(
        25.0f, 0.0f, 0.0f);

    // 左轮B路使用角速度闭环，角速度环输出单位为PWM
    Motor_TI.Motor_B.Set_Control_Method(
        Motor_TI_Control_Method_OMEGA);
    Motor_TI.Motor_B.PID_Omega.Init(
        125.0f, 1500.0f, 0.0f);
    Motor_TI.Motor_B.PID_Angle.Init(
        25.0f, 0.0f, 0.0f);

    Clear_Control();
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收数据缓冲区
 * @param Length 接收数据长度
 */
void Class_Chassis::UART_RxCpltCallback(
    uint8_t *Rx_Data,
    uint16_t Length)
{
    Motor_TI.UART_RxCpltCallback(Rx_Data, Length);
}

/**
 * @brief 1ms底盘解算与控制计算回调函数
 *
 * @note 执行顺序：
 *       1. 将底盘目标逆解为左右轮目标角速度；
 *       2. 将目标写入A、B电机对象；
 *       3. 处理TI最新反馈并计算双路速度环输出；
 *       4. 根据本周期反馈正解底盘实际速度。
 */
void Class_Chassis::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Kinematics_Inverse_Resolution();

    Output_To_Motor();
    Motor_TI.TIM_1ms_Calculate_PeriodElapsedCallback();

    Self_Resolution();
}

/**
 * @brief 1ms底盘控制命令发送回调函数
 *
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Chassis::TIM_1ms_Write_PeriodElapsedCallback()
{
    return (Motor_TI.TIM_1ms_Write_PeriodElapsedCallback());
}

/**
 * @brief 设置底盘控制状态
 *
 * @param __Control_State 底盘控制状态
 *
 * @note 切换为DISABLE时立即清空整车目标与双轮控制器，
 *       防止重新使能后恢复旧目标。
 */
void Class_Chassis::Set_Control_State(
    Enum_Chassis_Control_State __Control_State)
{
    Control_State = __Control_State;

    if (Control_State == Chassis_Control_State_DISABLE)
    {
        Clear_Control();
    }
}

/**
 * @brief 设置单轮线速度限幅
 *
 * @param __Wheel_Linear_Speed_Limit 单轮线速度限幅，单位m/s
 *
 * @note 参数小于等于0时关闭单轮线速度限幅。
 */
void Class_Chassis::Set_Wheel_Linear_Speed_Limit(
    float __Wheel_Linear_Speed_Limit)
{
    if (__Wheel_Linear_Speed_Limit > 0.0f)
    {
        Wheel_Linear_Speed_Limit =
            __Wheel_Linear_Speed_Limit;
    }
    else
    {
        Wheel_Linear_Speed_Limit = 0.0f;
    }
}

/**
 * @brief 清空底盘运动目标与双轮控制器
 *
 * @note 不清除左右轮累计编码位置，也不修改底盘结构参数。
 */
void Class_Chassis::Clear_Control()
{
    Target_Velocity_X = 0.0f;
    Target_Omega = 0.0f;

    Resolved_Target_Velocity_X = 0.0f;
    Resolved_Target_Omega = 0.0f;

    Target_Left_Wheel_Omega = 0.0f;
    Target_Right_Wheel_Omega = 0.0f;

    Motor_TI.Motor_A.Clear_Control();
    Motor_TI.Motor_B.Clear_Control();
}

/**
 * @brief 根据左右轮反馈角速度正解底盘速度
 *
 * @note Motor_A为右轮，Motor_B为左轮。
 * @note TI原始反馈使用电机机械坐标系；函数先通过左右轮符号映射
 *       转换到“驱动车体向前为正”的车轮逻辑坐标系。
 *
 * 差速底盘运动学正解：
 * Vx = (v_left + v_right) / 2
 * Omega = (v_right - v_left) / (2 * half_track)
 */
void Class_Chassis::Self_Resolution()
{
    /*
     * TI库返回的是各电机自身的机械角速度：
     * 从各自输出轴侧向电机内部看，逆时针为正。
     *
     * 左右电机镜像安装后，同号机械角速度会使车体原地旋转。
     * 因此先转换为车轮逻辑角速度：
     * 正值统一表示该车轮驱动车体沿X轴向前。
     */
    const float left_wheel_forward_omega =
        Motor_TI.Motor_B.Get_Now_Omega() *
        CHASSIS_LEFT_WHEEL_FORWARD_SIGN;

    const float right_wheel_forward_omega =
        Motor_TI.Motor_A.Get_Now_Omega() *
        CHASSIS_RIGHT_WHEEL_FORWARD_SIGN;

    const float left_wheel_linear_speed =
        left_wheel_forward_omega * Wheel_Radius;

    const float right_wheel_linear_speed =
        right_wheel_forward_omega * Wheel_Radius;

    Now_Velocity_X =
        (left_wheel_linear_speed +
         right_wheel_linear_speed) *
        0.5f;

    Now_Omega =
        (right_wheel_linear_speed -
         left_wheel_linear_speed) /
        (2.0f * Chassis_Half_Track);
}

/**
 * @brief 根据底盘目标速度逆解左右轮目标角速度
 *
 * @note 底盘Omega正方向为从车体上方向下看逆时针。
 *
 * 差速底盘运动学逆解：
 * v_left = Vx - Omega * half_track
 * v_right = Vx + Omega * half_track
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    float left_wheel_linear_speed =
        Target_Velocity_X -
        Target_Omega * Chassis_Half_Track;

    float right_wheel_linear_speed =
        Target_Velocity_X +
        Target_Omega * Chassis_Half_Track;

    /*
     * 若任一车轮超过设定线速度上限，则对左右轮速度整体等比缩放。
     * 这样可以在限幅后保持原始Vx与Omega的比例关系，
     * 避免单独截断某一车轮速度导致运动方向发生畸变。
     */
    if (Wheel_Linear_Speed_Limit > 0.0f)
    {
        float max_abs_wheel_linear_speed =
            fabsf(left_wheel_linear_speed);

        const float right_abs_wheel_linear_speed =
            fabsf(right_wheel_linear_speed);

        if (right_abs_wheel_linear_speed >
            max_abs_wheel_linear_speed)
        {
            max_abs_wheel_linear_speed =
                right_abs_wheel_linear_speed;
        }

        if (max_abs_wheel_linear_speed >
            Wheel_Linear_Speed_Limit)
        {
            const float scale =
                Wheel_Linear_Speed_Limit /
                max_abs_wheel_linear_speed;

            left_wheel_linear_speed *= scale;
            right_wheel_linear_speed *= scale;
        }
    }

    // 车轮线速度转换为输出轴目标角速度
    Target_Left_Wheel_Omega =
        left_wheel_linear_speed / Wheel_Radius;

    Target_Right_Wheel_Omega =
        right_wheel_linear_speed / Wheel_Radius;

    // 保存经过单轮限幅后真正能够执行的整车运动目标
    Resolved_Target_Velocity_X =
        (left_wheel_linear_speed +
         right_wheel_linear_speed) *
        0.5f;

    Resolved_Target_Omega =
        (right_wheel_linear_speed -
         left_wheel_linear_speed) /
        (2.0f * Chassis_Half_Track);
}

/**
 * @brief 将左右轮目标角速度输出到对应电机对象
 *
 * @note Target_Left_Wheel_Omega与Target_Right_Wheel_Omega均使用
 *       “驱动车体向前为正”的车轮逻辑坐标系。
 * @note 输出到TI电机对象前，必须乘对应FORWARD_SIGN转换为电机机械
 *       角速度目标。当前右轮A路需要取反，左轮B路保持原符号。
 */
void Class_Chassis::Output_To_Motor()
{
    switch (Control_State)
    {
        case (Chassis_Control_State_DISABLE):
        {
            Motor_TI.Motor_A.Set_Control_Enable(false);
            Motor_TI.Motor_B.Set_Control_Enable(false);

            Motor_TI.Motor_A.Set_Target_Omega(0.0f);
            Motor_TI.Motor_B.Set_Target_Omega(0.0f);
            break;
        }

        case (Chassis_Control_State_NORMAL):
        {
            Motor_TI.Motor_A.Set_Control_Enable(true);
            Motor_TI.Motor_B.Set_Control_Enable(true);

            Motor_TI.Motor_A.Set_Control_Method(
                Motor_TI_Control_Method_OMEGA);
            Motor_TI.Motor_B.Set_Control_Method(
                Motor_TI_Control_Method_OMEGA);

            /*
             * A路为右轮，B路为左轮。
             * 将“车体向前为正”的逻辑轮速目标转换为TI机械角速度目标。
             *
             * 正Vx且Omega为0时：
             * 左轮B路目标为正，输出轴逆时针；
             * 右轮A路目标为负，输出轴顺时针；
             * 两轮共同驱动车体直线前进。
             */
            Motor_TI.Motor_A.Set_Target_Omega(
                Target_Right_Wheel_Omega *
                CHASSIS_RIGHT_WHEEL_FORWARD_SIGN);

            Motor_TI.Motor_B.Set_Target_Omega(
                Target_Left_Wheel_Omega *
                CHASSIS_LEFT_WHEEL_FORWARD_SIGN);
            break;
        }

        default:
        {
            // 非法控制状态按失能处理
            Control_State = Chassis_Control_State_DISABLE;

            Motor_TI.Motor_A.Set_Control_Enable(false);
            Motor_TI.Motor_B.Set_Control_Enable(false);

            Motor_TI.Motor_A.Set_Target_Omega(0.0f);
            Motor_TI.Motor_B.Set_Target_Omega(0.0f);
            break;
        }
    }
}

/*----------------------------------------------------------------------------*/
