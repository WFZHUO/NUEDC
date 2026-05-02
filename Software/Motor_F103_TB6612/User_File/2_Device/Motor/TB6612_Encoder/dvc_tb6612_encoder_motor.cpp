/**
 * @file dvc_tb6612_encoder_motor.cpp
 * @author WangFonzhuo
 * @brief TB6612有刷直流编码电机驱动
 * @version 1.0
 * @date 2026-04-29 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_tb6612_encoder_motor.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化TB6612编码电机对象
 */
void Class_TB6612_Encoder_Motor::Init(TIM_HandleTypeDef *__PWM_TIM, uint32_t __PWM_Channel,
                                      TIM_HandleTypeDef *__Encoder_TIM,
                                      GPIO_TypeDef *__AIN1_GPIO, uint16_t __AIN1_Pin,
                                      GPIO_TypeDef *__AIN2_GPIO, uint16_t __AIN2_Pin,
                                      uint32_t __Encoder_Count_Per_Round,
                                      float __Control_Period_s,
                                      GPIO_TypeDef *__STBY_GPIO, uint16_t __STBY_Pin)
{
    PWM_TIM = __PWM_TIM;
    PWM_Channel = __PWM_Channel;
    Encoder_TIM = __Encoder_TIM;

    AIN1_GPIO = __AIN1_GPIO;
    AIN1_Pin = __AIN1_Pin;
    AIN2_GPIO = __AIN2_GPIO;
    AIN2_Pin = __AIN2_Pin;
    STBY_GPIO = __STBY_GPIO;
    STBY_Pin = __STBY_Pin;

    if (__Encoder_Count_Per_Round == 0)
    {
        Encoder_Count_Per_Round = 1;
    }
    else
    {
        Encoder_Count_Per_Round = __Encoder_Count_Per_Round;
    }

    if (__Control_Period_s <= 0.0f)
    {
        Control_Period_s = 0.001f;
    }
    else
    {
        Control_Period_s = __Control_Period_s;
    }

    if (PWM_TIM != nullptr)
    {
        PWM_Max = __HAL_TIM_GET_AUTORELOAD(PWM_TIM) + 1;
    }
    else
    {
        PWM_Max = 0;
    }

    Last_Encoder = 0;
    Encoder_Delta = 0;
    Position = 0;
    Now_Omega = 0.0f;
    Target_Omega = 0.0f;
    PWM_Out = 0;
    Is_Started = false;
    Loop_Enable = true;
    Status = TB6612_Motor_Status_DISABLE;
}

/**
 * @brief 刹车停止
 */
void Class_TB6612_Encoder_Motor::Brake()
{
    if ((AIN1_GPIO == nullptr) || (AIN2_GPIO == nullptr) || (PWM_TIM == nullptr))
    {
        return;
    }

    HAL_GPIO_WritePin(AIN1_GPIO, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_Channel, 0);

    PWM_Out = 0;
    Status = TB6612_Motor_Status_BRAKE;
}

/**
 * @brief 设置开环PWM占空比, 范围-1.0~1.0
 */
void Class_TB6612_Encoder_Motor::Set_PWM_Percent(float __Percent)
{
    Loop_Enable = true;

    __Percent = Constrain_Float(__Percent, -1.0f, 1.0f);
    Set_PWM(static_cast<int32_t>(__Percent * static_cast<float>(PWM_Max)));
}

/**
 * @brief 设置速度闭环目标
 */
void Class_TB6612_Encoder_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
    PID_Omega.Set_Target(Target_Omega);
    Loop_Enable = false;
}

/**
 * @brief 关闭速度闭环, 回到开环模式
 */
void Class_TB6612_Encoder_Motor::Close_Loop_Disable()
{
    Loop_Enable = true;
    PID_Omega.Set_Integral_Error(0.0f);
}

/**
 * @brief 定时更新回调函数
 */
void Class_TB6612_Encoder_Motor::TIM_Update_PeriodElapsedCallback()
{
    if (Encoder_TIM == nullptr)
    {
        return;
    }

    int16_t now_encoder = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(Encoder_TIM));
    int16_t delta = now_encoder - Last_Encoder;
    Last_Encoder = now_encoder;

    if (Encoder_Reverse == true)
    {
        delta = -delta;
    }

    Encoder_Delta = delta;
    Position += Encoder_Delta;

    Now_Omega = static_cast<float>(Encoder_Delta) * 6.28f / (static_cast<float>(Encoder_Count_Per_Round) * Control_Period_s);

    if (Loop_Enable == false)
    {
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        float tmp_pwm = PID_Omega.Get_Out();
        Set_PWM(static_cast<int32_t>(tmp_pwm));
    }
}

/**
 * @brief 启动PWM和编码器
 */
void Class_TB6612_Encoder_Motor::Start()
{
    if ((PWM_TIM == nullptr) || (Encoder_TIM == nullptr))
    {
        return;
    }

    if (STBY_GPIO != nullptr)
    {
        HAL_GPIO_WritePin(STBY_GPIO, STBY_Pin, GPIO_PIN_SET);
    }

    HAL_TIM_PWM_Start(PWM_TIM, PWM_Channel);
    HAL_TIM_Encoder_Start(Encoder_TIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_Channel, 0);
    __HAL_TIM_SET_COUNTER(Encoder_TIM, 0);

    Last_Encoder = 0;
    Encoder_Delta = 0;
    Position = 0;
    Now_Omega = 0.0f;
    Target_Omega = 0.0f;
    PWM_Out = 0;
    Is_Started = true;

    Brake();
}

/**
 * @brief 设置开环PWM比较值, 带方向
 */
void Class_TB6612_Encoder_Motor::Set_PWM(int32_t __PWM)
{
    if ((PWM_TIM == nullptr) || (AIN1_GPIO == nullptr) || (AIN2_GPIO == nullptr))
    {
        return;
    }

    if (Is_Started == false)
    {
        Start();
    }

    PWM_Out = Constrain_PWM(__PWM);

    bool tmp_forward = (PWM_Out >= 0);
    if (Motor_Reverse == true)
    {
        tmp_forward = !tmp_forward;
    }

    if (tmp_forward == true)
    {
        Set_Direction_Forward();
        Status = TB6612_Motor_Status_FORWARD;
    }
    else
    {
        Set_Direction_Reverse();
        Status = TB6612_Motor_Status_REVERSE;
    }

    uint32_t tmp_pwm = static_cast<uint32_t>((PWM_Out >= 0) ? PWM_Out : -PWM_Out);

    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_Channel, tmp_pwm);
}

/**
 * @brief 设置正转方向
 */
void Class_TB6612_Encoder_Motor::Set_Direction_Forward()
{
    HAL_GPIO_WritePin(AIN1_GPIO, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO, AIN2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 设置反转方向
 */
void Class_TB6612_Encoder_Motor::Set_Direction_Reverse()
{
    HAL_GPIO_WritePin(AIN1_GPIO, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO, AIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief PWM比较值限幅
 */
int32_t Class_TB6612_Encoder_Motor::Constrain_PWM(int32_t __PWM)
{
    int32_t max = static_cast<int32_t>(PWM_Max);

    if (__PWM > max)
    {
        return (max);
    }
    else if (__PWM < -max)
    {
        return (-max);
    }

    return (__PWM);
}

/**
 * @brief 浮点数限幅
 */
float Class_TB6612_Encoder_Motor::Constrain_Float(float Value, float Min, float Max)
{
    if (Value < Min)
    {
        return (Min);
    }
    else if (Value > Max)
    {
        return (Max);
    }

    return (Value);
}

/*----------------------------------------------------------------------------*/
