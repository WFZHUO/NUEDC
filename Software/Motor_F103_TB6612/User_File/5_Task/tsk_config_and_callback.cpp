/**
 * @file tsk_config_and_callback.cpp
 * @author WangFonzhuo
 * @brief 当成main.c来用
 * @version 1.0
 * @date 2025-12-30 26赛季定稿
 * @date 2026-04-18 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "alg_pid.h"
#include "dvc_tb6612_encoder_motor.h"
#include "dvc_serialplot.h"
#include "drv_tim.h"
#include "drv_uart.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

// TB6612编码电机对象
Class_TB6612_Encoder_Motor Motor;

// Serialplot对象
Class_Serialplot_UART Serialplot;
const char *Serialplot_Rx_Variable_List[] = {
    "op",
    "oi",
    "od",
};
// serialplot测试用变量
float Motor_Now_Omega = 0.0f;
float Motor_Target_Omega = 0.0f;
float Motor_PWM_Out = 0.0f;
float Motor_Now_Encoder = 0.0f;

/* Function prototypes -------------------------------------------------------*/

// 电机速度正弦波测试函数
static void Motor_Sine_Wave_Test();

/* Function definitions ------------------------------------------------------*/

/**
 * @brief UART1接收回调函数
 */
void UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Serialplot.UART_RxCpltCallback(Buffer, Length);

    switch (Serialplot.Get_Variable_Index())
    {
    case 0:
        Motor.PID_Omega.Set_K_P(Serialplot.Get_Variable_Value());
        break;

    case 1:
        Motor.PID_Omega.Set_K_I(Serialplot.Get_Variable_Value());
        break;

    case 2:
        Motor.PID_Omega.Set_K_D(Serialplot.Get_Variable_Value());
        break;

    default:
        break;
    }
}

/**
 * @brief 1ms任务回调函数
 */
void Task1ms_Callback()
{
    // 1ms任务

    // motor
    Motor_Sine_Wave_Test();
    Motor.TIM_Update_PeriodElapsedCallback();

    // serialplot
    Motor_Now_Omega = Motor.Get_Now_Omega();
    Motor_Target_Omega = Motor.Get_Target_Omega();
    Motor_PWM_Out = (float)Motor.Get_PWM();
    Motor_Now_Encoder = (float)Motor.Get_Position();
    Serialplot.TIM_1ms_Write_PeriodElapsedCallback();

    // 10ms任务
    static uint16_t mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

    }

    // 100ms任务
    static uint16_t mod100 = 0;
    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;

    }
}

/**
 * @brief 主程序任务初始化函数
 */
void Task_Init()
{
    // 初始化TIM1
    TIM_Init(&htim1, Task1ms_Callback);
    // 开启定时器任务调度
    HAL_TIM_Base_Start_IT(&htim1);

    // 初始化电机
    Motor.Init(&htim1, TIM_CHANNEL_1,
               &htim2,
               GPIOA, GPIO_PIN_12,
               GPIOA, GPIO_PIN_15,
               13 * 20 * 4,
               0.001f);

    Motor.PID_Omega.Init(24.0f, 0.0f, 0.0f, 0.0f,
                         (float)Motor.Get_PWM_Max(),
                         (float)Motor.Get_PWM_Max(),
                         (float)Motor.Get_PWM_Max(),
                         0.001f);

    Motor.Set_PWM_Percent(0.0f);

    // 初始化UART1
    UART_Init(&huart1, UART1_Callback);
    // 初始化Serialplot
    Serialplot.Init(&huart1,
                    Serialplot_Checksum_8_ENABLE,
                    3,
                    Serialplot_Rx_Variable_List,
                    Serialplot_Data_Type_FLOAT,
                    0xab);
    Serialplot.Set_Data(4,
                        &Motor_Now_Omega,
                        &Motor_Target_Omega,
                        &Motor_PWM_Out,
                        &Motor_Now_Encoder);

    // 设置初始化完成标志位
    init_finished = true;
}

/**
 * @brief 主程序任务循环函数
 */
void Task_Loop()
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
}

/**
 * @brief 电机速度正弦波测试函数
 */
static void Motor_Sine_Wave_Test()
{
    static uint16_t mod20 = 0;
    static uint16_t index = 0;

    const float amplitude = 100.0f;  // 正弦波幅值, 单位rad/s
    const float bias = 0.0f;        // 正弦波偏置, 单位rad/s

    static const float sine_table[100] =
    {
        0.0000f, 0.0628f, 0.1253f, 0.1874f, 0.2487f, 0.3090f, 0.3681f, 0.4258f, 0.4818f, 0.5358f,
        0.5878f, 0.6374f, 0.6845f, 0.7290f, 0.7705f, 0.8090f, 0.8443f, 0.8763f, 0.9048f, 0.9298f,
        0.9511f, 0.9686f, 0.9823f, 0.9921f, 0.9980f, 1.0000f, 0.9980f, 0.9921f, 0.9823f, 0.9686f,
        0.9511f, 0.9298f, 0.9048f, 0.8763f, 0.8443f, 0.8090f, 0.7705f, 0.7290f, 0.6845f, 0.6374f,
        0.5878f, 0.5358f, 0.4818f, 0.4258f, 0.3681f, 0.3090f, 0.2487f, 0.1874f, 0.1253f, 0.0628f,
        0.0000f, -0.0628f, -0.1253f, -0.1874f, -0.2487f, -0.3090f, -0.3681f, -0.4258f, -0.4818f, -0.5358f,
        -0.5878f, -0.6374f, -0.6845f, -0.7290f, -0.7705f, -0.8090f, -0.8443f, -0.8763f, -0.9048f, -0.9298f,
        -0.9511f, -0.9686f, -0.9823f, -0.9921f, -0.9980f, -1.0000f, -0.9980f, -0.9921f, -0.9823f, -0.9686f,
        -0.9511f, -0.9298f, -0.9048f, -0.8763f, -0.8443f, -0.8090f, -0.7705f, -0.7290f, -0.6845f, -0.6374f,
        -0.5878f, -0.5358f, -0.4818f, -0.4258f, -0.3681f, -0.3090f, -0.2487f, -0.1874f, -0.1253f, -0.0628f,
    };

    mod20++;
    if (mod20 < 20)
    {
        return;
    }
    mod20 = 0;

    Motor.Set_Target_Omega(bias + amplitude * sine_table[index]);

    index++;
    if (index >= 100)
    {
        index = 0;
    }
}

/*----------------------------------------------------------------------------*/
