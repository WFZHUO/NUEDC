/**
 * @file drv_uart.cpp
 * @author WangFonzhuo
 * @brief UART通用接口
 * @version 1.0
 * @date 2026-05-01 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// UART管理对象
Struct_UART_Manage_Object UART1_Manage_Object;
Struct_UART_Manage_Object UART2_Manage_Object;
Struct_UART_Manage_Object UART3_Manage_Object;

// UART接收双缓冲区
static uint8_t UART1_Rx_Buffer_0[UART_BUFFER_SIZE];
static uint8_t UART1_Rx_Buffer_1[UART_BUFFER_SIZE];
static uint8_t UART2_Rx_Buffer_0[UART_BUFFER_SIZE];
static uint8_t UART2_Rx_Buffer_1[UART_BUFFER_SIZE];
static uint8_t UART3_Rx_Buffer_0[UART_BUFFER_SIZE];
static uint8_t UART3_Rx_Buffer_1[UART_BUFFER_SIZE];

/* Function prototypes -------------------------------------------------------*/

static Struct_UART_Manage_Object *UART_Get_Manage_Object(UART_HandleTypeDef *huart);
static void UART_Start_Receive(Struct_UART_Manage_Object *uart_manage_object);
static void UART_Switch_Buffer(Struct_UART_Manage_Object *uart_manage_object);

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 获取UART管理对象
 */
static Struct_UART_Manage_Object *UART_Get_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        return (nullptr);
    }

#if defined(USART1)
    if (huart->Instance == USART1)
    {
        return (&UART1_Manage_Object);
    }
#endif

#if defined(USART2)
    if (huart->Instance == USART2)
    {
        return (&UART2_Manage_Object);
    }
#endif

#if defined(USART3)
    if (huart->Instance == USART3)
    {
        return (&UART3_Manage_Object);
    }
#endif

    return (nullptr);
}

/**
 * @brief 启动UART空闲中断DMA接收
 */
static void UART_Start_Receive(Struct_UART_Manage_Object *uart_manage_object)
{
    if ((uart_manage_object == nullptr) ||
        (uart_manage_object->UART_Handler == nullptr) ||
        (uart_manage_object->Rx_Buffer_Active == nullptr))
    {
        return;
    }

    if (uart_manage_object->UART_Handler->hdmarx != nullptr)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(uart_manage_object->UART_Handler,
                                     uart_manage_object->Rx_Buffer_Active,
                                     UART_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(uart_manage_object->UART_Handler->hdmarx, DMA_IT_HT);
    }
    else
    {
        HAL_UARTEx_ReceiveToIdle_IT(uart_manage_object->UART_Handler,
                                    uart_manage_object->Rx_Buffer_Active,
                                    UART_BUFFER_SIZE);
    }
}

/**
 * @brief 切换UART接收缓冲区
 */
static void UART_Switch_Buffer(Struct_UART_Manage_Object *uart_manage_object)
{
    if (uart_manage_object == nullptr)
    {
        return;
    }

    uart_manage_object->Rx_Buffer_Ready = uart_manage_object->Rx_Buffer_Active;

    if (uart_manage_object->Rx_Buffer_Active == uart_manage_object->Rx_Buffer_0)
    {
        uart_manage_object->Rx_Buffer_Active = uart_manage_object->Rx_Buffer_1;
    }
    else
    {
        uart_manage_object->Rx_Buffer_Active = uart_manage_object->Rx_Buffer_0;
    }
}

/**
 * @brief 初始化UART
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback_Function)
{
    Struct_UART_Manage_Object *uart_manage_object = UART_Get_Manage_Object(huart);

    if (uart_manage_object == nullptr)
    {
        return;
    }

    uart_manage_object->UART_Handler = huart;
    uart_manage_object->Callback_Function = Callback_Function;

#if defined(USART1)
    if (huart->Instance == USART1)
    {
        uart_manage_object->Rx_Buffer_0 = UART1_Rx_Buffer_0;
        uart_manage_object->Rx_Buffer_1 = UART1_Rx_Buffer_1;
    }
#endif

#if defined(USART2)
    if (huart->Instance == USART2)
    {
        uart_manage_object->Rx_Buffer_0 = UART2_Rx_Buffer_0;
        uart_manage_object->Rx_Buffer_1 = UART2_Rx_Buffer_1;
    }
#endif

#if defined(USART3)
    if (huart->Instance == USART3)
    {
        uart_manage_object->Rx_Buffer_0 = UART3_Rx_Buffer_0;
        uart_manage_object->Rx_Buffer_1 = UART3_Rx_Buffer_1;
    }
#endif

    uart_manage_object->Rx_Buffer_Active = uart_manage_object->Rx_Buffer_0;
    uart_manage_object->Rx_Buffer_Ready = nullptr;
    uart_manage_object->Rx_Time_Stamp = 0;

    UART_Start_Receive(uart_manage_object);
}

/**
 * @brief 重新初始化UART
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *uart_manage_object = UART_Get_Manage_Object(huart);

    if (uart_manage_object == nullptr)
    {
        return;
    }

    uart_manage_object->Rx_Buffer_Active = uart_manage_object->Rx_Buffer_0;
    uart_manage_object->Rx_Buffer_Ready = nullptr;

    UART_Start_Receive(uart_manage_object);
}

/**
 * @brief UART发送数据
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    if ((huart == nullptr) || (Data == nullptr) || (Length == 0))
    {
        return (HAL_ERROR);
    }

    if (huart->hdmatx != nullptr)
    {
        return (HAL_UART_Transmit_DMA(huart, Data, Length));
    }

    return (HAL_UART_Transmit(huart, Data, Length, 10));
}

/**
 * @brief HAL库UART接收DMA空闲中断
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Struct_UART_Manage_Object *uart_manage_object = UART_Get_Manage_Object(huart);

    if (uart_manage_object == nullptr)
    {
        return;
    }

    if (init_finished == false)
    {
        UART_Start_Receive(uart_manage_object);
        return;
    }

    UART_Switch_Buffer(uart_manage_object);
    uart_manage_object->Rx_Time_Stamp = HAL_GetTick();

    UART_Start_Receive(uart_manage_object);

    if (uart_manage_object->Callback_Function != nullptr)
    {
        uart_manage_object->Callback_Function(uart_manage_object->Rx_Buffer_Ready, Size);
    }
}

/**
 * @brief HAL库UART错误中断
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Reinit(huart);
}

/*----------------------------------------------------------------------------*/
