/**
 * @file dvc_dr16.cpp
 * @author WangFonzhuo
 * @brief DJI DR16遥控器驱动
 * @version 1.1
 * @date 2026-07-18
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "alg_basic.h"
#include <string.h>

/* Variables -----------------------------------------------------------------*/

Class_DR16 DR16;

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化DR16软件状态并绑定UART句柄
 * @param huart DR16使用的UART句柄
 * @note 不修改UART参数，不调用HAL_UART_Init/DeInit，不启动DMA接收
 */
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
    UART_Handler = huart;

    Flag = 0U;
    Pre_Flag = 0U;
    DR16_Status = DR16_Status_DISABLE;

    Reset_Data_To_Safe_State();
}

void Class_DR16::Judge_Switch(Enum_DR16_Switch_Status *Switch,
                              uint8_t Status,
                              uint8_t Pre_Status)
{
    if (Switch == nullptr)
    {
        return;
    }

    switch (Pre_Status)
    {
        case SWITCH_UP:
            if (Status == SWITCH_UP)
            {
                *Switch = DR16_Switch_Status_UP;
            }
            else if (Status == SWITCH_MIDDLE)
            {
                *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;
            }
            else
            {
                *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            }
            break;

        case SWITCH_MIDDLE:
            if (Status == SWITCH_UP)
            {
                *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            }
            else if (Status == SWITCH_MIDDLE)
            {
                *Switch = DR16_Switch_Status_MIDDLE;
            }
            else
            {
                *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
            }
            break;

        case SWITCH_DOWN:
            if (Status == SWITCH_UP)
            {
                *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
            }
            else if (Status == SWITCH_MIDDLE)
            {
                *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
            }
            else
            {
                *Switch = DR16_Switch_Status_DOWN;
            }
            break;

        default:
            *Switch = Get_Switch_Steady_State(Status);
            break;
    }
}

void Class_DR16::Judge_Key(Enum_DR16_Key_Status *Key,
                           uint8_t Status,
                           uint8_t Pre_Status)
{
    if (Key == nullptr)
    {
        return;
    }

    if (Pre_Status == KEY_FREE)
    {
        *Key = (Status == KEY_PRESSED) ?
               DR16_Key_Status_TRIG_FREE_PRESSED :
               DR16_Key_Status_FREE;
    }
    else
    {
        *Key = (Status == KEY_PRESSED) ?
               DR16_Key_Status_PRESSED :
               DR16_Key_Status_TRIG_PRESSED_FREE;
    }
}

Enum_DR16_Switch_Status Class_DR16::Get_Switch_Steady_State(uint8_t Status) const
{
    if (Status == SWITCH_MIDDLE)
    {
        return DR16_Switch_Status_MIDDLE;
    }

    if (Status == SWITCH_DOWN)
    {
        return DR16_Switch_Status_DOWN;
    }

    return DR16_Switch_Status_UP;
}

Enum_DR16_Key_Status Class_DR16::Get_Key_Steady_State(uint8_t Status) const
{
    return (Status == KEY_PRESSED) ?
           DR16_Key_Status_PRESSED :
           DR16_Key_Status_FREE;
}

bool Class_DR16::Data_Process(const uint8_t *Rx_Data)
{
    if (Rx_Data == nullptr)
    {
        return false;
    }

    Struct_DR16_UART_Data raw = {};

    raw.Channel_0 = (static_cast<uint16_t>(Rx_Data[0]) |
                    (static_cast<uint16_t>(Rx_Data[1]) << 8U)) & 0x07FFU;
    raw.Channel_1 = ((static_cast<uint16_t>(Rx_Data[1]) >> 3U) |
                    (static_cast<uint16_t>(Rx_Data[2]) << 5U)) & 0x07FFU;
    raw.Channel_2 = ((static_cast<uint16_t>(Rx_Data[2]) >> 6U) |
                    (static_cast<uint16_t>(Rx_Data[3]) << 2U) |
                    (static_cast<uint16_t>(Rx_Data[4]) << 10U)) & 0x07FFU;
    raw.Channel_3 = ((static_cast<uint16_t>(Rx_Data[4]) >> 1U) |
                    (static_cast<uint16_t>(Rx_Data[5]) << 7U)) & 0x07FFU;

    raw.Switch_1 = (Rx_Data[5] >> 6U) & 0x03U;
    raw.Switch_2 = (Rx_Data[5] >> 4U) & 0x03U;

    raw.Mouse_X = static_cast<int16_t>(static_cast<uint16_t>(Rx_Data[6]) |
                                      (static_cast<uint16_t>(Rx_Data[7]) << 8U));
    raw.Mouse_Y = static_cast<int16_t>(static_cast<uint16_t>(Rx_Data[8]) |
                                      (static_cast<uint16_t>(Rx_Data[9]) << 8U));
    raw.Mouse_Z = static_cast<int16_t>(static_cast<uint16_t>(Rx_Data[10]) |
                                      (static_cast<uint16_t>(Rx_Data[11]) << 8U));
    raw.Mouse_Left_Key = Rx_Data[12];
    raw.Mouse_Right_Key = Rx_Data[13];
    raw.Keyboard_Key = static_cast<uint16_t>(Rx_Data[14]) |
                       (static_cast<uint16_t>(Rx_Data[15]) << 8U);
    raw.Channel_Yaw = (static_cast<uint16_t>(Rx_Data[16]) |
                      (static_cast<uint16_t>(Rx_Data[17]) << 8U)) & 0x07FFU;

    if (raw.Channel_0 < 364U || raw.Channel_0 > 1684U ||
        raw.Channel_1 < 364U || raw.Channel_1 > 1684U ||
        raw.Channel_2 < 364U || raw.Channel_2 > 1684U ||
        raw.Channel_3 < 364U || raw.Channel_3 > 1684U ||
        raw.Channel_Yaw < 364U || raw.Channel_Yaw > 1684U)
    {
        return false;
    }

    if (raw.Switch_1 < SWITCH_UP || raw.Switch_1 > SWITCH_MIDDLE ||
        raw.Switch_2 < SWITCH_UP || raw.Switch_2 > SWITCH_MIDDLE)
    {
        return false;
    }

    if (raw.Mouse_Left_Key > KEY_PRESSED ||
        raw.Mouse_Right_Key > KEY_PRESSED)
    {
        return false;
    }

    Struct_DR16_Data temp_data = {};

    temp_data.Right_X = (static_cast<float>(raw.Channel_0) - ROCKER_OFFSET) / ROCKER_RANGE;
    temp_data.Right_Y = (static_cast<float>(raw.Channel_1) - ROCKER_OFFSET) / ROCKER_RANGE;
    temp_data.Left_X = (static_cast<float>(raw.Channel_2) - ROCKER_OFFSET) / ROCKER_RANGE;
    temp_data.Left_Y = (static_cast<float>(raw.Channel_3) - ROCKER_OFFSET) / ROCKER_RANGE;
    temp_data.Yaw = (static_cast<float>(raw.Channel_Yaw) - ROCKER_OFFSET) / ROCKER_RANGE;

    if (Basic_Math_Abs(temp_data.Right_X) > 1.0f ||
        Basic_Math_Abs(temp_data.Right_Y) > 1.0f ||
        Basic_Math_Abs(temp_data.Left_X) > 1.0f ||
        Basic_Math_Abs(temp_data.Left_Y) > 1.0f ||
        Basic_Math_Abs(temp_data.Yaw) > 1.0f)
    {
        return false;
    }

    if (Has_Valid_Frame)
    {
        Judge_Switch(&temp_data.Left_Switch, raw.Switch_1, Pre_UART_Rx_Data.Switch_1);
        Judge_Switch(&temp_data.Right_Switch, raw.Switch_2, Pre_UART_Rx_Data.Switch_2);

        Judge_Key(&temp_data.Mouse_Left_Key,
                  raw.Mouse_Left_Key,
                  Pre_UART_Rx_Data.Mouse_Left_Key);
        Judge_Key(&temp_data.Mouse_Right_Key,
                  raw.Mouse_Right_Key,
                  Pre_UART_Rx_Data.Mouse_Right_Key);

        for (uint8_t i = 0U; i < 16U; i++)
        {
            Judge_Key(&temp_data.Keyboard_Key[i],
                      (raw.Keyboard_Key >> i) & 0x01U,
                      (Pre_UART_Rx_Data.Keyboard_Key >> i) & 0x01U);
        }
    }
    else
    {
        temp_data.Left_Switch = Get_Switch_Steady_State(raw.Switch_1);
        temp_data.Right_Switch = Get_Switch_Steady_State(raw.Switch_2);
        temp_data.Mouse_Left_Key = Get_Key_Steady_State(raw.Mouse_Left_Key);
        temp_data.Mouse_Right_Key = Get_Key_Steady_State(raw.Mouse_Right_Key);

        for (uint8_t i = 0U; i < 16U; i++)
        {
            temp_data.Keyboard_Key[i] =
                Get_Key_Steady_State((raw.Keyboard_Key >> i) & 0x01U);
        }
    }

    temp_data.Mouse_X = static_cast<float>(raw.Mouse_X) / 32768.0f;
    temp_data.Mouse_Y = static_cast<float>(raw.Mouse_Y) / 32768.0f;
    temp_data.Mouse_Z = static_cast<float>(raw.Mouse_Z) / 32768.0f;

    Data = temp_data;
    Pre_UART_Rx_Data = raw;
    Has_Valid_Frame = true;

    return true;
}

/**
 * @brief UART接收完成回调
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (UART_Handler == nullptr ||
        Rx_Data == nullptr ||
        Length != DR16_RX_FRAME_LENGTH)
    {
        return;
    }

    if (Data_Process(Rx_Data))
    {
        Flag++;
        DR16_Status = DR16_Status_ENABLE;
    }
}

/**
 * @brief 每50ms检查DR16是否在线
 */
void Class_DR16::TIM_50ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        DR16_Status = DR16_Status_DISABLE;
        Reset_Data_To_Safe_State();
    }
    else
    {
        DR16_Status = DR16_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_DR16::Reset_Data_To_Safe_State()
{
    memset(&Data, 0, sizeof(Data));
    memset(&Pre_UART_Rx_Data, 0, sizeof(Pre_UART_Rx_Data));

    Data.Left_Switch = DR16_Switch_Status_UP;
    Data.Right_Switch = DR16_Switch_Status_UP;
    Data.Mouse_Left_Key = DR16_Key_Status_FREE;
    Data.Mouse_Right_Key = DR16_Key_Status_FREE;

    for (uint8_t i = 0U; i < 16U; i++)
    {
        Data.Keyboard_Key[i] = DR16_Key_Status_FREE;
    }

    Has_Valid_Frame = false;
}

/*----------------------------------------------------------------------------*/
