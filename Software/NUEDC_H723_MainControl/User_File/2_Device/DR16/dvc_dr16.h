/**
 * @file dvc_dr16.h
 * @author WangFonzhuo
 * @brief DJI DR16遥控器驱动，负责数据解析、合法性校验与在线状态维护
 * @version 1.1
 * @date 2026-07-18
 * @note UART参数、GPIO、DMA与中断全部由CubeMX和drv_uart底层负责配置
 */

#ifndef DVC_DR16_H
#define DVC_DR16_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

// 拨动开关原始位置
#define SWITCH_UP       (1U)
#define SWITCH_DOWN     (2U)
#define SWITCH_MIDDLE   (3U)

// 按键原始状态
#define KEY_FREE        (0U)
#define KEY_PRESSED     (1U)

// 键盘按键索引
#define KEY_W           (0U)
#define KEY_S           (1U)
#define KEY_A           (2U)
#define KEY_D           (3U)
#define KEY_SHIFT       (4U)
#define KEY_CTRL        (5U)
#define KEY_Q           (6U)
#define KEY_E           (7U)
#define KEY_R           (8U)
#define KEY_F           (9U)
#define KEY_G           (10U)
#define KEY_Z           (11U)
#define KEY_X           (12U)
#define KEY_C           (13U)
#define KEY_V           (14U)
#define KEY_B           (15U)

/* Exported types ------------------------------------------------------------*/

enum Enum_DR16_Status
{
    DR16_Status_DISABLE = 0,
    DR16_Status_ENABLE,
};

enum Enum_DR16_Switch_Status
{
    DR16_Switch_Status_UP = 0,
    DR16_Switch_Status_TRIG_UP_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_UP,
    DR16_Switch_Status_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_DOWN,
    DR16_Switch_Status_TRIG_DOWN_MIDDLE,
    DR16_Switch_Status_DOWN,
};

enum Enum_DR16_Key_Status
{
    DR16_Key_Status_FREE = 0,
    DR16_Key_Status_TRIG_FREE_PRESSED,
    DR16_Key_Status_TRIG_PRESSED_FREE,
    DR16_Key_Status_PRESSED,
};

/**
 * @brief DR16解包后的原始数据
 */
struct Struct_DR16_UART_Data
{
    uint16_t Channel_0;
    uint16_t Channel_1;
    uint16_t Channel_2;
    uint16_t Channel_3;
    uint8_t Switch_1;
    uint8_t Switch_2;
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint8_t Mouse_Left_Key;
    uint8_t Mouse_Right_Key;
    uint16_t Keyboard_Key;
    uint16_t Channel_Yaw;
};

/**
 * @brief DR16处理后的对外数据
 * @note 摇杆与拨轮归一化到[-1, 1]
 */
struct Struct_DR16_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    Enum_DR16_Switch_Status Left_Switch;
    Enum_DR16_Switch_Status Right_Switch;
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;
    Enum_DR16_Key_Status Mouse_Left_Key;
    Enum_DR16_Key_Status Mouse_Right_Key;
    Enum_DR16_Key_Status Keyboard_Key[16];
    float Yaw;
};

class Class_DR16
{
public:
    /**
     * @brief 初始化DR16软件状态并绑定UART句柄
     * @param huart DR16使用的UART句柄
     * @note 本函数不会修改UART参数，不会调用HAL_UART_Init/DeInit，也不会启动DMA接收
     */
    void Init(UART_HandleTypeDef *huart);

    inline Enum_DR16_Status Get_DR16_Status() const { return DR16_Status; }
    inline float Get_Right_X() const { return Data.Right_X; }
    inline float Get_Right_Y() const { return Data.Right_Y; }
    inline float Get_Left_X() const { return Data.Left_X; }
    inline float Get_Left_Y() const { return Data.Left_Y; }
    inline Enum_DR16_Switch_Status Get_Left_Switch() const { return Data.Left_Switch; }
    inline Enum_DR16_Switch_Status Get_Right_Switch() const { return Data.Right_Switch; }
    inline float Get_Mouse_X() const { return Data.Mouse_X; }
    inline float Get_Mouse_Y() const { return Data.Mouse_Y; }
    inline float Get_Mouse_Z() const { return Data.Mouse_Z; }
    inline Enum_DR16_Key_Status Get_Mouse_Left_Key() const { return Data.Mouse_Left_Key; }
    inline Enum_DR16_Key_Status Get_Mouse_Right_Key() const { return Data.Mouse_Right_Key; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_W() const { return Data.Keyboard_Key[KEY_W]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_S() const { return Data.Keyboard_Key[KEY_S]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_A() const { return Data.Keyboard_Key[KEY_A]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_D() const { return Data.Keyboard_Key[KEY_D]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Shift() const { return Data.Keyboard_Key[KEY_SHIFT]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Ctrl() const { return Data.Keyboard_Key[KEY_CTRL]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Q() const { return Data.Keyboard_Key[KEY_Q]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_E() const { return Data.Keyboard_Key[KEY_E]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_R() const { return Data.Keyboard_Key[KEY_R]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_F() const { return Data.Keyboard_Key[KEY_F]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_G() const { return Data.Keyboard_Key[KEY_G]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_Z() const { return Data.Keyboard_Key[KEY_Z]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_X() const { return Data.Keyboard_Key[KEY_X]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_C() const { return Data.Keyboard_Key[KEY_C]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_V() const { return Data.Keyboard_Key[KEY_V]; }
    inline Enum_DR16_Key_Status Get_Keyboard_Key_B() const { return Data.Keyboard_Key[KEY_B]; }
    inline float Get_Yaw() const { return Data.Yaw; }

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_50ms_Alive_PeriodElapsedCallback();

protected:
    static constexpr uint16_t DR16_RX_FRAME_LENGTH = 18U;
    static constexpr float ROCKER_OFFSET = 1024.0f;
    static constexpr float ROCKER_RANGE = 660.0f;

    // 仅记录DR16绑定的UART，不负责配置或启动该UART
    UART_HandleTypeDef *UART_Handler = nullptr;

    Struct_DR16_UART_Data Pre_UART_Rx_Data = {};
    volatile uint32_t Flag = 0U;
    uint32_t Pre_Flag = 0U;
    bool Has_Valid_Frame = false;

    volatile Enum_DR16_Status DR16_Status = DR16_Status_DISABLE;
    Struct_DR16_Data Data = {};

    void Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
    void Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
    Enum_DR16_Switch_Status Get_Switch_Steady_State(uint8_t Status) const;
    Enum_DR16_Key_Status Get_Key_Steady_State(uint8_t Status) const;
    bool Data_Process(const uint8_t *Rx_Data);
    void Reset_Data_To_Safe_State();
};

/* Exported variables --------------------------------------------------------*/

extern Class_DR16 DR16;

#endif /* DVC_DR16_H */

/*----------------------------------------------------------------------------*/
