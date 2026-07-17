/**
 * @file dvc_motor_ti.cpp
 * @author WangFonzhuo
 * @brief MSPM0 TI双路有刷电机驱动板
 * @version 1.0
 * @date 2026-07-16 27赛季
 * @note 黑盒库！
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_ti.h"
#include "alg_basic.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/

static constexpr uint8_t MOTOR_TI_PROTOCOL_SOF_0 = 0xA5U;
static constexpr uint8_t MOTOR_TI_PROTOCOL_SOF_1 = 0x5AU;

static constexpr uint8_t MOTOR_TI_CMD_SET_PWM = 0x10U;
static constexpr uint8_t MOTOR_TI_CMD_STOP = 0x11U;
static constexpr uint8_t MOTOR_TI_CMD_PING = 0x12U;
static constexpr uint8_t MOTOR_TI_CMD_FEEDBACK = 0x90U;
static constexpr uint8_t MOTOR_TI_CMD_PONG = 0x91U;

static constexpr uint8_t MOTOR_TI_SET_PWM_PAYLOAD_LENGTH = 4U;
static constexpr uint8_t MOTOR_TI_FEEDBACK_PAYLOAD_LENGTH = 13U;
static constexpr uint8_t MOTOR_TI_PROTOCOL_MAX_PAYLOAD_LENGTH = 16U;
static constexpr uint8_t MOTOR_TI_PROTOCOL_BUFFER_SIZE = 32U;

static constexpr uint8_t MOTOR_TI_STATUS_COMMAND_ACTIVE = (1U << 0U);
static constexpr uint8_t MOTOR_TI_STATUS_COMMAND_TIMEOUT = (1U << 1U);

static constexpr float MOTOR_TI_TWO_PI = 6.28318530717958647692f;

/* Types ---------------------------------------------------------------------*/

/**
 * @brief TI驱动板协议解析状态
 */
enum Enum_Motor_TI_Parser_State
{
    Motor_TI_Parser_State_WAIT_SOF_0 = 0,
    Motor_TI_Parser_State_WAIT_SOF_1,
    Motor_TI_Parser_State_WAIT_SEQUENCE,
    Motor_TI_Parser_State_WAIT_COMMAND,
    Motor_TI_Parser_State_WAIT_LENGTH,
    Motor_TI_Parser_State_WAIT_PAYLOAD,
    Motor_TI_Parser_State_WAIT_CRC_LOW,
    Motor_TI_Parser_State_WAIT_CRC_HIGH,
};

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化单路电机
 *
 * @param __Motor_ID 电机通道
 * @param __Encoder_Num_Per_Round 输出轴每圈软件计数
 * @param __Direction 电机逻辑方向
 */
void Class_Motor_TI_Channel::Init(Enum_Motor_TI_ID __Motor_ID,
                                  float __Encoder_Num_Per_Round,
                                  Enum_Motor_TI_Direction __Direction)
{
    Motor_ID = __Motor_ID;
    Direction = __Direction;

    if (__Encoder_Num_Per_Round > 0.0f)
    {
        Encoder_Num_Per_Round = __Encoder_Num_Per_Round;
    }
    else
    {
        Encoder_Num_Per_Round = MOTOR_TI_ENCODER_NUM_PER_ROUND;
    }

    Control_Method = Motor_TI_Control_Method_PWM;
    Control_Enable = true;
    PWM_Limit = MOTOR_TI_PWM_MAX;
    PWM_Change_Max_Per_MS = 0.0f;

    Clear();
}

/**
 * @brief 获取完整反馈数据
 *
 * @return Struct_Motor_TI_Rx_Data 电机反馈数据
 */
Struct_Motor_TI_Rx_Data Class_Motor_TI_Channel::Get_Rx_Data() const
{
    Struct_Motor_TI_Rx_Data rx_data = {};

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    rx_data.Now_Encoder = Rx_Data.Now_Encoder;
    rx_data.Pre_Encoder = Rx_Data.Pre_Encoder;
    rx_data.Total_Encoder = Rx_Data.Total_Encoder;
    rx_data.Now_Angle = Rx_Data.Now_Angle;
    rx_data.Now_Omega = Rx_Data.Now_Omega;
    rx_data.Encoder_Init_Flag = Rx_Data.Encoder_Init_Flag;

    __set_PRIMASK(primask);

    return (rx_data);
}

/**
 * @brief 设置电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_Motor_TI_Channel::Set_Control_Method(Enum_Motor_TI_Control_Method __Control_Method)
{
    if (Control_Method == __Control_Method)
    {
        return;
    }

    Control_Method = __Control_Method;

    // 切换控制模式时清空旧模式目标与输出, 防止恢复旧目标导致突跳
    Clear_Control();
}

/**
 * @brief 设置电机逻辑方向
 *
 * @param __Direction 电机逻辑方向
 */
void Class_Motor_TI_Channel::Set_Direction(
    Enum_Motor_TI_Direction __Direction)
{
    if (Direction == __Direction)
    {
        return;
    }

    Direction = __Direction;

    // Direction现在只表示PWM输出极性，不影响编码器坐标系
    Clear_Control();
}

/**
 * @brief 设置控制使能状态
 *
 * @param __Control_Enable 控制使能状态
 */
void Class_Motor_TI_Channel::Set_Control_Enable(bool __Control_Enable)
{
    Control_Enable = __Control_Enable;

    if (!Control_Enable)
    {
        Clear_Control();
    }
}

/**
 * @brief 设置输出轴每圈软件计数
 *
 * @param __Encoder_Num_Per_Round 输出轴每圈软件计数
 */
void Class_Motor_TI_Channel::Set_Encoder_Num_Per_Round(float __Encoder_Num_Per_Round)
{
    if (__Encoder_Num_Per_Round <= 0.0f)
    {
        return;
    }

    Encoder_Num_Per_Round = __Encoder_Num_Per_Round;
    Clear_Position();
}

/**
 * @brief 设置PWM输出限幅
 *
 * @param __PWM_Limit PWM输出限幅
 */
void Class_Motor_TI_Channel::Set_PWM_Limit(float __PWM_Limit)
{
    __PWM_Limit = Basic_Math_Abs(__PWM_Limit);
    PWM_Limit = Basic_Math_Constrain(__PWM_Limit, 0.0f, MOTOR_TI_PWM_MAX);
    Out = Basic_Math_Constrain(Out, -PWM_Limit, PWM_Limit);
}

/**
 * @brief 设置每毫秒PWM最大变化量
 *
 * @param __PWM_Change_Max_Per_MS 每毫秒PWM最大变化量, 0为不限制
 */
void Class_Motor_TI_Channel::Set_PWM_Change_Max_Per_MS(float __PWM_Change_Max_Per_MS)
{
    if (__PWM_Change_Max_Per_MS > 0.0f)
    {
        PWM_Change_Max_Per_MS = __PWM_Change_Max_Per_MS;
    }
    else
    {
        PWM_Change_Max_Per_MS = 0.0f;
    }
}

/**
 * @brief 清空PID、目标值与控制输出
 *
 * @note 不清除累计位置
 */
void Class_Motor_TI_Channel::Clear_Control()
{
    PID_Angle.Clear();
    PID_Omega.Clear();

    Target_PWM = 0.0f;
    Target_Omega = 0.0f;
    Target_Angle = Get_Now_Angle();

    Feedforward_PWM = 0.0f;
    Feedforward_Omega = 0.0f;

    Out = 0.0f;
}

/**
 * @brief 将当前位置设置为逻辑零点
 */
void Class_Motor_TI_Channel::Clear_Position()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (Rx_Data.Encoder_Init_Flag)
    {
        Rx_Data.Pre_Encoder = Rx_Data.Now_Encoder;
    }

    Rx_Data.Total_Encoder = 0;
    Rx_Data.Now_Angle = 0.0f;

    __set_PRIMASK(primask);

    Target_Angle = 0.0f;
    PID_Angle.Clear();
}

/**
 * @brief 完整重置单路电机对象
 */
void Class_Motor_TI_Channel::Clear()
{
    Motor_Status = Motor_TI_Status_DISABLE;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    Rx_Data.Now_Encoder = 0;
    Rx_Data.Pre_Encoder = 0;
    Rx_Data.Total_Encoder = 0;
    Rx_Data.Now_Angle = 0.0f;
    Rx_Data.Now_Omega = 0.0f;
    Rx_Data.Encoder_Init_Flag = false;

    __set_PRIMASK(primask);

    // 反馈清零后再清控制量, 保证目标角度同步回到0
    Clear_Control();
}

/**
 * @brief 设置电机在线状态
 *
 * @param __Status 电机在线状态
 */
void Class_Motor_TI_Channel::Set_Status(Enum_Motor_TI_Status __Status)
{
    if (Motor_Status == __Status)
    {
        return;
    }

    Motor_Status = __Status;

    if (Motor_Status == Motor_TI_Status_DISABLE)
    {
        Clear_Control();
    }
}

/**
 * @brief 处理单路电机原始反馈
 *
 * @param Raw_Encoder TI板上传的原始累计编码值
 * @param Raw_Speed_CPS TI板上传的编码计数速度, counts/s
 */
void Class_Motor_TI_Channel::Data_Process(int32_t Raw_Encoder,
                                          int16_t Raw_Speed_CPS)
{
    // TI板原始编码器方向已经符合机械方向定义：
    // 从输出轴向内看，逆时针为正，顺时针为负
    int32_t logical_speed_cps = (int32_t)Raw_Speed_CPS;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!Rx_Data.Encoder_Init_Flag)
    {
        // 第一帧只建立编码器基准
        Rx_Data.Encoder_Init_Flag = true;
        Rx_Data.Now_Encoder = Raw_Encoder;
        Rx_Data.Pre_Encoder = Raw_Encoder;
        Rx_Data.Total_Encoder = 0;
    }
    else
    {
        uint32_t delta_encoder_unsigned =
            (uint32_t)Raw_Encoder - (uint32_t)Rx_Data.Pre_Encoder;

        int32_t delta_encoder;
        memcpy(&delta_encoder,
               &delta_encoder_unsigned,
               sizeof(delta_encoder));

        // 不要根据Direction翻转编码器增量
        Rx_Data.Total_Encoder += (int64_t)delta_encoder;

        Rx_Data.Now_Encoder = Raw_Encoder;
        Rx_Data.Pre_Encoder = Raw_Encoder;
    }

    Rx_Data.Now_Angle =
        (float)Rx_Data.Total_Encoder *
        MOTOR_TI_TWO_PI /
        Encoder_Num_Per_Round;

    Rx_Data.Now_Omega =
        (float)logical_speed_cps *
        MOTOR_TI_TWO_PI /
        Encoder_Num_Per_Round;

    __set_PRIMASK(primask);
}

/**
 * @brief 1ms控制计算
 */
void Class_Motor_TI_Channel::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    if (!Control_Enable || Motor_Status == Motor_TI_Status_DISABLE)
    {
        // 离线或失能时持续清空目标与输出, 防止恢复通信后执行旧目标
        Clear_Control();
        return;
    }

    Struct_Motor_TI_Rx_Data rx_data = Get_Rx_Data();
    float target_out = 0.0f;

    switch (Control_Method)
    {
        case (Motor_TI_Control_Method_PWM):
        {
            target_out = Target_PWM + Feedforward_PWM;
            break;
        }

        case (Motor_TI_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(rx_data.Now_Omega);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            target_out = PID_Omega.Get_Out() + Feedforward_PWM;
            break;
        }

        case (Motor_TI_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(rx_data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(rx_data.Now_Omega);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            target_out = PID_Omega.Get_Out() + Feedforward_PWM;
            break;
        }

        default:
        {
            target_out = 0.0f;
            break;
        }
    }

    target_out = Basic_Math_Constrain(target_out, -PWM_Limit, PWM_Limit);

    if (PWM_Change_Max_Per_MS > 0.0f)
    {
        float delta_out = target_out - Out;
        delta_out = Basic_Math_Constrain(delta_out,
                                         -PWM_Change_Max_Per_MS,
                                         PWM_Change_Max_Per_MS);
        Out += delta_out;
    }
    else
    {
        Out = target_out;
    }

    Out = Basic_Math_Constrain(Out, -PWM_Limit, PWM_Limit);
}

/**
 * @brief 获取发送给TI板的物理PWM
 *
 * @return int16_t 物理PWM
 */
int16_t Class_Motor_TI_Channel::Get_Physical_Out_PWM() const
{
    float physical_out = Out;

    if (Direction == Motor_TI_Direction_REVERSE)
    {
        physical_out = -physical_out;
    }

    physical_out = Basic_Math_Constrain(physical_out,
                                        -MOTOR_TI_PWM_MAX,
                                        MOTOR_TI_PWM_MAX);

    return ((int16_t)physical_out);
}

/**
 * @brief 初始化TI驱动板
 *
 * @param huart 与TI驱动板连接的UART
 * @param __Encoder_Num_Per_Round 输出轴每圈软件计数
 * @param __Direction_A A路电机逻辑方向
 * @param __Direction_B B路电机逻辑方向
 */
void Class_Motor_TI::Init(UART_HandleTypeDef *huart,
                          float __Encoder_Num_Per_Round,
                          Enum_Motor_TI_Direction __Direction_A,
                          Enum_Motor_TI_Direction __Direction_B)
{
    if (huart == nullptr)
    {
        return;
    }

    UART_Handler = huart;

    Motor_A.Init(Motor_TI_ID_A,
                 __Encoder_Num_Per_Round,
                 __Direction_A);

    Motor_B.Init(Motor_TI_ID_B,
                 __Encoder_Num_Per_Round,
                 __Direction_B);

    Reset_Parser();
    Last_Rx_Callback_Time_MS = HAL_GetTick();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    Feedback_Encoder_A = 0;
    Feedback_Encoder_B = 0;
    Feedback_Speed_CPS_A = 0;
    Feedback_Speed_CPS_B = 0;
    Feedback_Status = 0U;
    Feedback_Sequence = 0U;
    Feedback_Pending = false;

    Remote_Status = 0U;
    Last_Feedback_Sequence = 0U;
    Has_Valid_Feedback = false;

    Valid_Feedback_Count = 0U;
    CRC_Error_Count = 0U;
    Sequence_Error_Count = 0U;
    Pong_Count = 0U;
    Tx_Busy_Count = 0U;
    Tx_Error_Count = 0U;
    Tx_Building = false;

    __set_PRIMASK(primask);

    Offline_Count_MS = 0U;
    Offline_Timeout_MS = MOTOR_TI_OFFLINE_TIMEOUT_MS;

    Tx_Sequence = 0U;
    Last_Rx_Sequence = 0U;
    Has_Previous_Rx_Sequence = false;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收数据缓冲区
 * @param Length 数据长度
 */
void Class_Motor_TI::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Rx_Data == nullptr || Length == 0U)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    /*
     * 上一次只收到半帧且间隔已经过长时丢弃旧半帧,
     * 防止残缺帧与后续新帧拼接。
     */
    if (Parser_State != Motor_TI_Parser_State_WAIT_SOF_0 &&
        (uint32_t)(now - Last_Rx_Callback_Time_MS) > MOTOR_TI_RX_FRAME_TIMEOUT_MS)
    {
        Reset_Parser();
    }

    Last_Rx_Callback_Time_MS = now;

    for (uint16_t i = 0U; i < Length; i++)
    {
        Parse_Byte(Rx_Data[i]);
    }
}

/**
 * @brief 1ms定时器中断计算回调函数
 *
 * @note 处理反馈、更新在线状态并计算A/B两路控制输出
 */
void Class_Motor_TI::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    bool feedback_updated = Process_Feedback_Mailbox();

    if (feedback_updated)
    {
        Offline_Count_MS = 0U;
        Has_Valid_Feedback = true;
    }
    else if (Offline_Count_MS < UINT32_MAX)
    {
        Offline_Count_MS++;
    }

    Update_Online_Status();

    Motor_A.TIM_1ms_Calculate_PeriodElapsedCallback();
    Motor_B.TIM_1ms_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 1ms定时器中断发送回调函数
 *
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Motor_TI::TIM_1ms_Write_PeriodElapsedCallback()
{
    if (!Is_Online())
    {
        // 收不到TI反馈时禁止继续盲控, 只发送安全零输出
        return (Send_PWM_Command(0, 0));
    }

    return (Send_PWM_Command(Motor_A.Get_Physical_Out_PWM(),
                             Motor_B.Get_Physical_Out_PWM()));
}

/**
 * @brief 直接发送双路PWM命令
 *
 * @param PWM_A A路物理PWM
 * @param PWM_B B路物理PWM
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Motor_TI::Send_PWM_Command(int16_t PWM_A, int16_t PWM_B)
{
    PWM_A = Clamp_PWM(PWM_A);
    PWM_B = Clamp_PWM(PWM_B);

    uint8_t payload[MOTOR_TI_SET_PWM_PAYLOAD_LENGTH];

    Put_I16_BE(&payload[0], PWM_A);
    Put_I16_BE(&payload[2], PWM_B);

    return (Send_Frame(MOTOR_TI_CMD_SET_PWM,
                       payload,
                       MOTOR_TI_SET_PWM_PAYLOAD_LENGTH));
}

/**
 * @brief 发送停车命令
 *
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Motor_TI::Stop()
{
    Motor_A.Clear_Control();
    Motor_B.Clear_Control();

    return (Send_Frame(MOTOR_TI_CMD_STOP, nullptr, 0U));
}

/**
 * @brief 发送Ping命令
 *
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Motor_TI::Ping()
{
    return (Send_Frame(MOTOR_TI_CMD_PING, nullptr, 0U));
}

/**
 * @brief 判断TI驱动板是否在线
 *
 * @return true TI驱动板在线
 * @return false TI驱动板离线
 */
bool Class_Motor_TI::Is_Online() const
{
    return (Has_Valid_Feedback && Offline_Count_MS <= Offline_Timeout_MS);
}

/**
 * @brief 判断TI驱动板是否处于命令有效状态
 *
 * @return true 命令有效且未超时
 * @return false 命令无效或已经超时
 */
bool Class_Motor_TI::Get_Remote_Command_Active() const
{
    return ((Remote_Status & MOTOR_TI_STATUS_COMMAND_ACTIVE) != 0U);
}

/**
 * @brief 判断TI驱动板是否进入命令超时停车
 *
 * @return true TI驱动板已经超时停车
 * @return false TI驱动板未处于超时状态
 */
bool Class_Motor_TI::Get_Remote_Command_Timeout() const
{
    return ((Remote_Status & MOTOR_TI_STATUS_COMMAND_TIMEOUT) != 0U);
}

/**
 * @brief 获取通信统计信息
 *
 * @return Struct_Motor_TI_Protocol_Statistics 通信统计信息
 */
Struct_Motor_TI_Protocol_Statistics Class_Motor_TI::Get_Protocol_Statistics() const
{
    Struct_Motor_TI_Protocol_Statistics statistics = {};

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    statistics.Valid_Feedback_Count = Valid_Feedback_Count;
    statistics.CRC_Error_Count = CRC_Error_Count;
    statistics.Sequence_Error_Count = Sequence_Error_Count;
    statistics.Pong_Count = Pong_Count;
    statistics.Tx_Busy_Count = Tx_Busy_Count;
    statistics.Tx_Error_Count = Tx_Error_Count;

    __set_PRIMASK(primask);

    return (statistics);
}

/**
 * @brief 重置协议解析器
 */
void Class_Motor_TI::Reset_Parser()
{
    Parser_State = Motor_TI_Parser_State_WAIT_SOF_0;
    Rx_Frame_Index = 0U;
    Rx_Command = 0U;
    Rx_Payload_Length = 0U;
}

/**
 * @brief 逐字节解析协议
 *
 * @param Byte 当前接收字节
 */
void Class_Motor_TI::Parse_Byte(uint8_t Byte)
{
    switch (Parser_State)
    {
        case (Motor_TI_Parser_State_WAIT_SOF_0):
        {
            if (Byte == MOTOR_TI_PROTOCOL_SOF_0)
            {
                Rx_Frame_Buffer[0] = Byte;
                Parser_State = Motor_TI_Parser_State_WAIT_SOF_1;
            }
            break;
        }

        case (Motor_TI_Parser_State_WAIT_SOF_1):
        {
            if (Byte == MOTOR_TI_PROTOCOL_SOF_1)
            {
                Rx_Frame_Buffer[1] = Byte;
                Rx_Frame_Index = 2U;
                Parser_State = Motor_TI_Parser_State_WAIT_SEQUENCE;
            }
            else if (Byte == MOTOR_TI_PROTOCOL_SOF_0)
            {
                // A5 A5 5A时保留第二个A5作为新帧头
                Rx_Frame_Buffer[0] = Byte;
            }
            else
            {
                Reset_Parser();
            }
            break;
        }

        case (Motor_TI_Parser_State_WAIT_SEQUENCE):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;
            Parser_State = Motor_TI_Parser_State_WAIT_COMMAND;
            break;
        }

        case (Motor_TI_Parser_State_WAIT_COMMAND):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;
            Rx_Command = Byte;
            Parser_State = Motor_TI_Parser_State_WAIT_LENGTH;
            break;
        }

        case (Motor_TI_Parser_State_WAIT_LENGTH):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;
            Rx_Payload_Length = Byte;

            if (Rx_Payload_Length > MOTOR_TI_PROTOCOL_MAX_PAYLOAD_LENGTH)
            {
                Reset_Parser();
            }
            else if (Rx_Payload_Length == 0U)
            {
                Parser_State = Motor_TI_Parser_State_WAIT_CRC_LOW;
            }
            else
            {
                Parser_State = Motor_TI_Parser_State_WAIT_PAYLOAD;
            }
            break;
        }

        case (Motor_TI_Parser_State_WAIT_PAYLOAD):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;

            if (Rx_Frame_Index >= (uint8_t)(5U + Rx_Payload_Length))
            {
                Parser_State = Motor_TI_Parser_State_WAIT_CRC_LOW;
            }
            break;
        }

        case (Motor_TI_Parser_State_WAIT_CRC_LOW):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;
            Parser_State = Motor_TI_Parser_State_WAIT_CRC_HIGH;
            break;
        }

        case (Motor_TI_Parser_State_WAIT_CRC_HIGH):
        {
            Rx_Frame_Buffer[Rx_Frame_Index++] = Byte;

            uint16_t received_crc =
                (uint16_t)Rx_Frame_Buffer[Rx_Frame_Index - 2U] |
                ((uint16_t)Rx_Frame_Buffer[Rx_Frame_Index - 1U] << 8U);

            uint16_t calculated_crc =
                CRC16_Modbus(Rx_Frame_Buffer,
                             (uint16_t)(Rx_Frame_Index - 2U));

            if (received_crc == calculated_crc)
            {
                Accept_Frame(Rx_Frame_Buffer[2],
                             Rx_Command,
                             &Rx_Frame_Buffer[5],
                             Rx_Payload_Length);
            }
            else
            {
                CRC_Error_Count++;
            }

            Reset_Parser();
            break;
        }

        default:
        {
            Reset_Parser();
            break;
        }
    }
}

/**
 * @brief 处理通过CRC校验的完整帧
 *
 * @param Sequence 帧序号
 * @param Command 命令字
 * @param Payload 数据载荷
 * @param Length 数据载荷长度
 */
void Class_Motor_TI::Accept_Frame(uint8_t Sequence,
                                  uint8_t Command,
                                  const uint8_t *Payload,
                                  uint8_t Length)
{
    Update_Rx_Sequence(Sequence);

    if (Command == MOTOR_TI_CMD_FEEDBACK)
    {
        if (Payload == nullptr || Length != MOTOR_TI_FEEDBACK_PAYLOAD_LENGTH)
        {
            return;
        }

        int32_t encoder_a = Get_I32_BE(&Payload[0]);
        int32_t encoder_b = Get_I32_BE(&Payload[4]);
        int16_t speed_cps_a = Get_I16_BE(&Payload[8]);
        int16_t speed_cps_b = Get_I16_BE(&Payload[10]);
        uint8_t status = Payload[12];

        // UART回调只发布原始反馈, 浮点换算与PID统一放在1ms控制回调
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        Feedback_Encoder_A = encoder_a;
        Feedback_Encoder_B = encoder_b;
        Feedback_Speed_CPS_A = speed_cps_a;
        Feedback_Speed_CPS_B = speed_cps_b;
        Feedback_Status = status;
        Feedback_Sequence = Sequence;
        Feedback_Pending = true;

        Valid_Feedback_Count++;

        __set_PRIMASK(primask);
    }
    else if (Command == MOTOR_TI_CMD_PONG)
    {
        if (Length == 0U)
        {
            Pong_Count++;
        }
    }
}

/**
 * @brief 检查TI发送帧序号
 *
 * @param Sequence 当前帧序号
 */
void Class_Motor_TI::Update_Rx_Sequence(uint8_t Sequence)
{
    if (Has_Previous_Rx_Sequence)
    {
        uint8_t sequence_delta = (uint8_t)(Sequence - Last_Rx_Sequence);

        if (sequence_delta != 1U)
        {
            Sequence_Error_Count++;
        }
    }
    else
    {
        Has_Previous_Rx_Sequence = true;
    }

    Last_Rx_Sequence = Sequence;
}

/**
 * @brief 取出并处理反馈邮箱
 *
 * @return true 本周期处理了新反馈
 * @return false 本周期没有新反馈
 */
bool Class_Motor_TI::Process_Feedback_Mailbox()
{
    bool feedback_pending;
    int32_t encoder_a = 0;
    int32_t encoder_b = 0;
    int16_t speed_cps_a = 0;
    int16_t speed_cps_b = 0;
    uint8_t status = 0U;
    uint8_t sequence = 0U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    feedback_pending = Feedback_Pending;

    if (feedback_pending)
    {
        encoder_a = Feedback_Encoder_A;
        encoder_b = Feedback_Encoder_B;
        speed_cps_a = Feedback_Speed_CPS_A;
        speed_cps_b = Feedback_Speed_CPS_B;
        status = Feedback_Status;
        sequence = Feedback_Sequence;

        Feedback_Pending = false;
    }

    __set_PRIMASK(primask);

    if (!feedback_pending)
    {
        return (false);
    }

    Motor_A.Data_Process(encoder_a, speed_cps_a);
    Motor_B.Data_Process(encoder_b, speed_cps_b);

    Remote_Status = status;
    Last_Feedback_Sequence = sequence;

    return (true);
}

/**
 * @brief 更新A/B电机在线状态
 */
void Class_Motor_TI::Update_Online_Status()
{
    Enum_Motor_TI_Status status = Is_Online() ?
                                  Motor_TI_Status_ENABLE :
                                  Motor_TI_Status_DISABLE;

    Motor_A.Set_Status(status);
    Motor_B.Set_Status(status);
}

/**
 * @brief 发送一帧TI协议数据
 *
 * @param Command 命令字
 * @param Payload 数据载荷
 * @param Length 数据载荷长度
 * @return true 发送启动成功
 * @return false UART忙或发送失败
 */
bool Class_Motor_TI::Send_Frame(uint8_t Command,
                                const uint8_t *Payload,
                                uint8_t Length)
{
    if (UART_Handler == nullptr ||
        Length > MOTOR_TI_PROTOCOL_MAX_PAYLOAD_LENGTH ||
        (Length > 0U && Payload == nullptr))
    {
        return (false);
    }

    /*
     * 防止主循环与定时器中断同时组帧并竞争Tx_Sequence。
     * UART通用驱动还会继续检查对应UART的DMA发送状态。
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (Tx_Building)
    {
        Tx_Busy_Count++;
        __set_PRIMASK(primask);
        return (false);
    }

    Tx_Building = true;
    uint8_t sequence = Tx_Sequence;

    __set_PRIMASK(primask);

    /*
     * UART通用驱动会先将该临时帧复制到RAM_D2发送缓冲区,
     * 因此这里可以安全使用栈上局部数组。
     */
    uint8_t frame[MOTOR_TI_PROTOCOL_BUFFER_SIZE];
    uint8_t index = 0U;

    frame[index++] = MOTOR_TI_PROTOCOL_SOF_0;
    frame[index++] = MOTOR_TI_PROTOCOL_SOF_1;
    frame[index++] = sequence;
    frame[index++] = Command;
    frame[index++] = Length;

    for (uint8_t i = 0U; i < Length; i++)
    {
        frame[index++] = Payload[i];
    }

    uint16_t crc = CRC16_Modbus(frame, index);

    frame[index++] = (uint8_t)(crc & 0xFFU);
    frame[index++] = (uint8_t)(crc >> 8U);

    HAL_StatusTypeDef tx_status = (HAL_StatusTypeDef)
        UART_Transmit_Data(UART_Handler, frame, index);

    primask = __get_PRIMASK();
    __disable_irq();

    if (tx_status == HAL_OK)
    {
        Tx_Sequence++;
    }
    else if (tx_status == HAL_BUSY)
    {
        Tx_Busy_Count++;
    }
    else
    {
        Tx_Error_Count++;
    }

    Tx_Building = false;

    __set_PRIMASK(primask);

    return (tx_status == HAL_OK);
}

/**
 * @brief Modbus CRC16
 *
 * @param Data 数据地址
 * @param Length 数据长度
 * @return uint16_t CRC16结果
 */
uint16_t Class_Motor_TI::CRC16_Modbus(const uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr)
    {
        return (0U);
    }

    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < Length; i++)
    {
        crc ^= Data[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return (crc);
}

/**
 * @brief PWM限幅
 *
 * @param PWM PWM值
 * @return int16_t 限幅后的PWM值
 */
int16_t Class_Motor_TI::Clamp_PWM(int16_t PWM)
{
    return (Basic_Math_Constrain(PWM,
                                 (int16_t)-MOTOR_TI_PWM_MAX,
                                 (int16_t)MOTOR_TI_PWM_MAX));
}

/**
 * @brief 写入16位大端有符号数
 *
 * @param Buffer 数据缓冲区
 * @param Value 需要写入的数值
 */
void Class_Motor_TI::Put_I16_BE(uint8_t *Buffer, int16_t Value)
{
    uint16_t raw_value = (uint16_t)Value;

    Buffer[0] = (uint8_t)(raw_value >> 8U);
    Buffer[1] = (uint8_t)raw_value;
}

/**
 * @brief 读取16位大端有符号数
 *
 * @param Buffer 数据缓冲区
 * @return int16_t 解析结果
 */
int16_t Class_Motor_TI::Get_I16_BE(const uint8_t *Buffer)
{
    uint16_t raw_value =
        ((uint16_t)Buffer[0] << 8U) |
        (uint16_t)Buffer[1];

    int16_t value;
    memcpy(&value, &raw_value, sizeof(value));

    return (value);
}

/**
 * @brief 读取32位大端有符号数
 *
 * @param Buffer 数据缓冲区
 * @return int32_t 解析结果
 */
int32_t Class_Motor_TI::Get_I32_BE(const uint8_t *Buffer)
{
    uint32_t raw_value =
        ((uint32_t)Buffer[0] << 24U) |
        ((uint32_t)Buffer[1] << 16U) |
        ((uint32_t)Buffer[2] << 8U) |
        (uint32_t)Buffer[3];

    int32_t value;
    memcpy(&value, &raw_value, sizeof(value));

    return (value);
}

/*----------------------------------------------------------------------------*/
