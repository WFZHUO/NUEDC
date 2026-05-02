/**
 * @file dvc_tb6612_encoder_motor.h
 * @author WangFonzhuo
 * @brief TB6612有刷直流编码电机驱动
 * @version 1.0
 * @date 2026-04-29 27赛季
 */

#ifndef DVC_TB6612_ENCODER_MOTOR_H
#define DVC_TB6612_ENCODER_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "alg_pid.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TB6612电机运行状态
 */
typedef enum
{
    TB6612_Motor_Status_DISABLE = 0,
    TB6612_Motor_Status_BRAKE,
    TB6612_Motor_Status_FORWARD,
    TB6612_Motor_Status_REVERSE,
} Enum_TB6612_Motor_Status;

/**
 * @brief TB6612编码电机对象
 */
class Class_TB6612_Encoder_Motor
{
public:
    // 速度PID对象
    Class_PID PID_Omega;

    /**
     * @brief 初始化TB6612编码电机对象
     *
     * @param __PWM_TIM PWM定时器
     * @param __PWM_Channel PWM通道
     * @param __Encoder_TIM 编码器模式定时器
     * @param __AIN1_GPIO AIN1端口
     * @param __AIN1_Pin AIN1引脚
     * @param __AIN2_GPIO AIN2端口
     * @param __AIN2_Pin AIN2引脚
     * @param __Encoder_Count_Per_Round 输出轴每圈编码器计数, 为编码器线数*减速比*4
     * @param __Control_Period_s TIM_Update_PeriodElapsedCallback调用周期, 单位s
     * @param __STBY_GPIO STBY端口, 若硬件直接上拉则填nullptr
     * @param __STBY_Pin STBY引脚, 若硬件直接上拉则填0
     */
    void Init(TIM_HandleTypeDef *__PWM_TIM, uint32_t __PWM_Channel,
              TIM_HandleTypeDef *__Encoder_TIM,
              GPIO_TypeDef *__AIN1_GPIO, uint16_t __AIN1_Pin,
              GPIO_TypeDef *__AIN2_GPIO, uint16_t __AIN2_Pin,
              uint32_t __Encoder_Count_Per_Round,
              float __Control_Period_s,
              GPIO_TypeDef *__STBY_GPIO = nullptr, uint16_t __STBY_Pin = 0);

    /**
     * @brief 刹车停止
     */
    void Brake();

    /**
     * @brief 设置开环PWM占空比, 范围-1.0~1.0
     */
    void Set_PWM_Percent(float __Percent);

    /**
     * @brief 设置速度闭环目标
     *
     * @param __Target_Omega 目标速度, 单位rad/s
     */
    void Set_Target_Omega(float __Target_Omega);

    /**
     * @brief 关闭速度闭环, 回到开环模式
     */
    void Close_Loop_Disable();

    /**
     * @brief 定时更新回调函数
     */
    void TIM_Update_PeriodElapsedCallback();

    /**
     * @brief 设置编码器方向反向
     */
    inline void Set_Encoder_Reverse(bool __Encoder_Reverse);

    /**
     * @brief 设置电机输出方向反向
     */
    inline void Set_Motor_Reverse(bool __Motor_Reverse);

    /**
     * @brief 获取电机状态
     */
    inline Enum_TB6612_Motor_Status Get_Status();

    /**
     * @brief 获取当前位置计数
     */
    inline int32_t Get_Position();

    /**
     * @brief 获取当前周期增量计数
     */
    inline int16_t Get_Delta();

    /**
     * @brief 获取当前速度rad/s
     */
    inline float Get_Now_Omega();

    /**
     * @brief 获取目标速度rad/s
     */
    inline float Get_Target_Omega();

    /**
     * @brief 获取当前PWM输出比较值
     */
    inline int32_t Get_PWM();

    /**
    * @brief 获取PWM最大比较值
    */
    inline uint32_t Get_PWM_Max();

protected:
    // 硬件绑定及初始化配置
    TIM_HandleTypeDef *PWM_TIM = nullptr;
    TIM_HandleTypeDef *Encoder_TIM = nullptr;
    uint32_t PWM_Channel = 0;
    uint32_t PWM_Max = 0;

    GPIO_TypeDef *AIN1_GPIO = nullptr;
    GPIO_TypeDef *AIN2_GPIO = nullptr;
    GPIO_TypeDef *STBY_GPIO = nullptr;
    uint16_t AIN1_Pin = 0;
    uint16_t AIN2_Pin = 0;
    uint16_t STBY_Pin = 0;

    // 输出轴每圈编码器计数
    uint32_t Encoder_Count_Per_Round = 1;
    // 定时更新周期
    float Control_Period_s = 0.001f;

    // 上一次的编码器计数
    int16_t Last_Encoder = 0;
    // 本周期编码器增量
    int16_t Encoder_Delta = 0;
    // 当前位置计数
    int32_t Position = 0;
    // 输出轴当前速度rad/s
    float Now_Omega = 0.0f;
    // 输出轴目标速度rad/s
    float Target_Omega = 0.0f;
    // 当前PWM输出比较值
    int32_t PWM_Out = 0;

    // 是否已启动
    bool Is_Started = false;
    // 是否开环模式
    bool Loop_Enable = true;
    // 编码器方向反向标志位
    bool Encoder_Reverse = false;
    // 电机输出方向反向标志位
    bool Motor_Reverse = false;

    // 电机状态
    Enum_TB6612_Motor_Status Status = TB6612_Motor_Status_DISABLE;

    // 启动PWM和编码器
    void Start();

    // 设置开环PWM比较值, 带方向
    void Set_PWM(int32_t __PWM);

    // 设置正转方向
    void Set_Direction_Forward();

    // 设置反转方向
    void Set_Direction_Reverse();

    // PWM限制函数
    int32_t Constrain_PWM(int32_t __PWM);

    // 浮点数限制函数
    float Constrain_Float(float Value, float Min, float Max);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 设置编码器方向反向
 * @param __Encoder_Reverse true为反向，false为正向
 */
inline void Class_TB6612_Encoder_Motor::Set_Encoder_Reverse(bool __Encoder_Reverse)
{
    Encoder_Reverse = __Encoder_Reverse;
}

/**
 * @brief 设置电机输出方向反向
 * @param __Motor_Reverse true为反向，false为正向
 */
inline void Class_TB6612_Encoder_Motor::Set_Motor_Reverse(bool __Motor_Reverse)
{
    Motor_Reverse = __Motor_Reverse;
}

/**
 * @brief 获取电机状态
 */
inline Enum_TB6612_Motor_Status Class_TB6612_Encoder_Motor::Get_Status()
{
    return (Status);
}

/**
 * @brief 获取当前位置计数
 */
inline int32_t Class_TB6612_Encoder_Motor::Get_Position()
{
    return (Position);
}

/**
 * @brief 获取当前周期增量计数
 */
inline int16_t Class_TB6612_Encoder_Motor::Get_Delta()
{
    return (Encoder_Delta);
}

/**
 * @brief 获取当前速度rad/s
 */
inline float Class_TB6612_Encoder_Motor::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取目标速度rad/s
 */
inline float Class_TB6612_Encoder_Motor::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取当前PWM输出比较值
 */
inline int32_t Class_TB6612_Encoder_Motor::Get_PWM()
{
    return (PWM_Out);
}

/**
 * @brief 获取PWM最大比较值
 */
inline uint32_t Class_TB6612_Encoder_Motor::Get_PWM_Max()
{
    return (PWM_Max);
}

#endif /* DVC_TB6612_ENCODER_MOTOR_H */

/*----------------------------------------------------------------------------*/
