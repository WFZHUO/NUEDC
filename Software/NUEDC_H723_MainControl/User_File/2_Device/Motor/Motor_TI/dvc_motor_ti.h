/**
 * @file dvc_motor_ti.h
 * @author WangFonzhuo
 * @brief MSPM0 TI双路有刷电机驱动板
 * @version 1.0
 * @date 2026-07-16 27赛季
 * @note 黑盒库！
 */

#ifndef DVC_MOTOR_TI_H
#define DVC_MOTOR_TI_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_uart.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// TI驱动板PWM指令最大值
#define MOTOR_TI_PWM_MAX 4000.0f

// MG310输出轴每圈软件累计计数
#define MOTOR_TI_ENCODER_NUM_PER_ROUND 1040.0f

// 默认反馈离线判定时间, 单位ms
#define MOTOR_TI_OFFLINE_TIMEOUT_MS 10U

// 协议半帧接收超时时间, 单位ms
#define MOTOR_TI_RX_FRAME_TIMEOUT_MS 5U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TI电机状态
 */
enum Enum_Motor_TI_Status
{
    Motor_TI_Status_DISABLE = 0,
    Motor_TI_Status_ENABLE,
};

/**
 * @brief TI电机通道
 */
enum Enum_Motor_TI_ID
{
    Motor_TI_ID_A = 0,
    Motor_TI_ID_B,
};

/**
 * @brief TI电机控制方式
 */
enum Enum_Motor_TI_Control_Method
{
    Motor_TI_Control_Method_PWM = 0,
    Motor_TI_Control_Method_OMEGA,
    Motor_TI_Control_Method_ANGLE,
};

/**
 * @brief TI电机逻辑方向
 */
enum Enum_Motor_TI_Direction
{
    Motor_TI_Direction_NORMAL = 0,
    Motor_TI_Direction_REVERSE,
};

/**
 * @brief TI单路电机反馈数据
 */
struct Struct_Motor_TI_Rx_Data
{
    // TI板上传的原始累计编码值
    int32_t Now_Encoder;
    int32_t Pre_Encoder;

    // H723从第一帧反馈开始累计的编码值
    int64_t Total_Encoder;

    // 输出轴角度与角速度
    float Now_Angle;
    float Now_Omega;

    bool Encoder_Init_Flag;
};

/**
 * @brief TI驱动板通信统计信息
 */
struct Struct_Motor_TI_Protocol_Statistics
{
    uint32_t Valid_Feedback_Count;
    uint32_t CRC_Error_Count;
    uint32_t Sequence_Error_Count;
    uint32_t Pong_Count;
    uint32_t Tx_Busy_Count;
    uint32_t Tx_Error_Count;
};

class Class_Motor_TI;

/**
 * @brief TI驱动板单路电机控制类
 */
class Class_Motor_TI_Channel
{
public:
    // 角度环输出单位为rad/s
    Class_PID PID_Angle;

    // 角速度环输出单位为PWM
    Class_PID PID_Omega;

    /**
     * @brief 获取电机状态
     *
     * @return Enum_Motor_TI_Status 电机状态
     */
    inline Enum_Motor_TI_Status Get_Status() const;

    /**
     * @brief 获取电机通道
     *
     * @return Enum_Motor_TI_ID 电机通道
     */
    inline Enum_Motor_TI_ID Get_Motor_ID() const;

    /**
     * @brief 获取电机控制方式
     *
     * @return Enum_Motor_TI_Control_Method 电机控制方式
     */
    inline Enum_Motor_TI_Control_Method Get_Control_Method() const;

    /**
     * @brief 获取电机逻辑方向
     *
     * @return Enum_Motor_TI_Direction 电机逻辑方向
     */
    inline Enum_Motor_TI_Direction Get_Direction() const;

    /**
     * @brief 获取完整反馈数据
     *
     * @return Struct_Motor_TI_Rx_Data 电机反馈数据
     */
    Struct_Motor_TI_Rx_Data Get_Rx_Data() const;

    /**
     * @brief 获取TI板上传的原始累计编码值
     *
     * @return int32_t 原始累计编码值
     */
    inline int32_t Get_Now_Encoder() const;

    /**
     * @brief 获取H723侧累计编码值
     *
     * @return int64_t 累计编码值
     */
    inline int64_t Get_Total_Encoder() const;

    /**
     * @brief 获取当前输出轴角度
     *
     * @return float 当前角度, rad
     */
    inline float Get_Now_Angle() const;

    /**
     * @brief 获取当前输出轴角速度
     *
     * @return float 当前角速度, rad/s
     */
    inline float Get_Now_Omega() const;

    /**
     * @brief 获取目标PWM
     *
     * @return float 目标PWM
     */
    inline float Get_Target_PWM() const;

    /**
     * @brief 获取目标角速度
     *
     * @return float 目标角速度, rad/s
     */
    inline float Get_Target_Omega() const;

    /**
     * @brief 获取目标角度
     *
     * @return float 目标角度, rad
     */
    inline float Get_Target_Angle() const;

    /**
     * @brief 获取PWM前馈值
     *
     * @return float PWM前馈值
     */
    inline float Get_Feedforward_PWM() const;

    /**
     * @brief 获取角速度前馈值
     *
     * @return float 角速度前馈值, rad/s
     */
    inline float Get_Feedforward_Omega() const;

    /**
     * @brief 获取当前控制输出
     *
     * @return float 当前PWM输出
     */
    inline float Get_Out() const;

    /**
     * @brief 获取输出轴每圈软件计数
     *
     * @return float 输出轴每圈软件计数
     */
    inline float Get_Encoder_Num_Per_Round() const;

    /**
     * @brief 获取PWM输出限幅
     *
     * @return float PWM输出限幅
     */
    inline float Get_PWM_Limit() const;

    /**
     * @brief 获取每毫秒PWM最大变化量
     *
     * @return float 每毫秒PWM最大变化量
     */
    inline float Get_PWM_Change_Max_Per_MS() const;

    /**
     * @brief 获取控制使能状态
     *
     * @return true 控制使能
     * @return false 控制失能
     */
    inline bool Get_Control_Enable() const;

    /**
     * @brief 设置电机控制方式
     *
     * @param __Control_Method 电机控制方式
     */
    void Set_Control_Method(Enum_Motor_TI_Control_Method __Control_Method);

    /**
     * @brief 设置电机逻辑方向
     *
     * @param __Direction 电机逻辑方向
     */
    void Set_Direction(Enum_Motor_TI_Direction __Direction);

    /**
     * @brief 设置控制使能状态
     *
     * @param __Control_Enable 控制使能状态
     */
    void Set_Control_Enable(bool __Control_Enable);

    /**
     * @brief 设置目标PWM
     *
     * @param __Target_PWM 目标PWM
     */
    inline void Set_Target_PWM(float __Target_PWM);

    /**
     * @brief 设置目标角速度
     *
     * @param __Target_Omega 目标角速度, rad/s
     */
    inline void Set_Target_Omega(float __Target_Omega);

    /**
     * @brief 设置目标角度
     *
     * @param __Target_Angle 目标角度, rad
     */
    inline void Set_Target_Angle(float __Target_Angle);

    /**
     * @brief 设置PWM前馈值
     *
     * @param __Feedforward_PWM PWM前馈值
     */
    inline void Set_Feedforward_PWM(float __Feedforward_PWM);

    /**
     * @brief 设置角速度前馈值
     *
     * @param __Feedforward_Omega 角速度前馈值, rad/s
     */
    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    /**
     * @brief 设置输出轴每圈软件计数
     *
     * @param __Encoder_Num_Per_Round 输出轴每圈软件计数
     */
    void Set_Encoder_Num_Per_Round(float __Encoder_Num_Per_Round);

    /**
     * @brief 设置PWM输出限幅
     *
     * @param __PWM_Limit PWM输出限幅
     */
    void Set_PWM_Limit(float __PWM_Limit);

    /**
     * @brief 设置每毫秒PWM最大变化量
     *
     * @param __PWM_Change_Max_Per_MS 每毫秒PWM最大变化量, 0为不限制
     */
    void Set_PWM_Change_Max_Per_MS(float __PWM_Change_Max_Per_MS);

    /**
     * @brief 清空PID、目标值与控制输出
     *
     * @note 不清除累计位置
     */
    void Clear_Control();

    /**
     * @brief 将当前位置设置为逻辑零点
     */
    void Clear_Position();

    /**
     * @brief 完整重置单路电机对象
     */
    void Clear();

protected:
    friend class Class_Motor_TI;

    Enum_Motor_TI_Status Motor_Status = Motor_TI_Status_DISABLE;
    Enum_Motor_TI_ID Motor_ID = Motor_TI_ID_A;
    Enum_Motor_TI_Control_Method Control_Method = Motor_TI_Control_Method_PWM;
    Enum_Motor_TI_Direction Direction = Motor_TI_Direction_NORMAL;

    float Encoder_Num_Per_Round = MOTOR_TI_ENCODER_NUM_PER_ROUND;

    volatile Struct_Motor_TI_Rx_Data Rx_Data = {};

    bool Control_Enable = true;

    // 控制目标
    float Target_PWM = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Angle = 0.0f;

    // 控制前馈
    float Feedforward_PWM = 0.0f;
    float Feedforward_Omega = 0.0f;

    float PWM_Limit = MOTOR_TI_PWM_MAX;
    float PWM_Change_Max_Per_MS = 0.0f;
    float Out = 0.0f;

private:
    // 初始化单路电机
    void Init(Enum_Motor_TI_ID __Motor_ID,
              float __Encoder_Num_Per_Round,
              Enum_Motor_TI_Direction __Direction);

    // 设置电机在线状态
    void Set_Status(Enum_Motor_TI_Status __Status);

    // 处理单路电机原始反馈
    void Data_Process(int32_t Raw_Encoder, int16_t Raw_Speed_CPS);

    // 1ms控制计算
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // 获取发送给TI板的物理PWM
    int16_t Get_Physical_Out_PWM() const;
};

/**
 * @brief MSPM0 TI双路有刷电机驱动板设备类
 */
class Class_Motor_TI
{
public:
    Class_Motor_TI_Channel Motor_A;
    Class_Motor_TI_Channel Motor_B;

    /**
     * @brief 初始化TI驱动板
     *
     * @param huart 与TI驱动板连接的UART
     * @param __Encoder_Num_Per_Round 输出轴每圈软件计数
     * @param __Direction_A A路电机逻辑方向
     * @param __Direction_B B路电机逻辑方向
     */
    void Init(UART_HandleTypeDef *huart,
              float __Encoder_Num_Per_Round = MOTOR_TI_ENCODER_NUM_PER_ROUND,
              Enum_Motor_TI_Direction __Direction_A = Motor_TI_Direction_NORMAL,
              Enum_Motor_TI_Direction __Direction_B = Motor_TI_Direction_NORMAL);

    /**
     * @brief UART通信接收回调函数
     *
     * @param Rx_Data 接收数据缓冲区
     * @param Length 数据长度
     */
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    /**
     * @brief 1ms定时器中断计算回调函数
     *
     * @note 处理反馈、更新在线状态并计算A/B两路控制输出
     */
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    /**
     * @brief 1ms定时器中断发送回调函数
     *
     * @return true 发送启动成功
     * @return false UART忙或发送失败
     */
    bool TIM_1ms_Write_PeriodElapsedCallback();

    /**
     * @brief 直接发送双路PWM命令
     *
     * @param PWM_A A路物理PWM
     * @param PWM_B B路物理PWM
     * @return true 发送启动成功
     * @return false UART忙或发送失败
     * @note 该函数绕过PID与逻辑方向, 主要用于底层通信测试
     */
    bool Send_PWM_Command(int16_t PWM_A, int16_t PWM_B);

    /**
     * @brief 发送停车命令
     *
     * @return true 发送启动成功
     * @return false UART忙或发送失败
     */
    bool Stop();

    /**
     * @brief 发送Ping命令
     *
     * @return true 发送启动成功
     * @return false UART忙或发送失败
     */
    bool Ping();

    /**
     * @brief 判断TI驱动板是否在线
     *
     * @return true TI驱动板在线
     * @return false TI驱动板离线
     */
    bool Is_Online() const;

    /**
     * @brief 获取TI驱动板状态字节
     *
     * @return uint8_t 状态字节
     */
    inline uint8_t Get_Remote_Status() const;

    /**
     * @brief 判断TI驱动板是否处于命令有效状态
     *
     * @return true 命令有效且未超时
     * @return false 命令无效或已经超时
     */
    bool Get_Remote_Command_Active() const;

    /**
     * @brief 判断TI驱动板是否进入命令超时停车
     *
     * @return true TI驱动板已经超时停车
     * @return false TI驱动板未处于超时状态
     */
    bool Get_Remote_Command_Timeout() const;

    /**
     * @brief 获取最后一帧反馈序号
     *
     * @return uint8_t 最后一帧反馈序号
     */
    inline uint8_t Get_Last_Feedback_Sequence() const;

    /**
     * @brief 获取反馈离线计时
     *
     * @return uint32_t 离线计时, ms
     */
    inline uint32_t Get_Offline_Count_MS() const;

    /**
     * @brief 获取通信统计信息
     *
     * @return Struct_Motor_TI_Protocol_Statistics 通信统计信息
     */
    Struct_Motor_TI_Protocol_Statistics Get_Protocol_Statistics() const;

    /**
     * @brief 设置反馈离线判定时间
     *
     * @param __Offline_Timeout_MS 反馈离线判定时间, ms
     */
    inline void Set_Offline_Timeout_MS(uint32_t __Offline_Timeout_MS);

    /**
     * @brief 获取反馈离线判定时间
     *
     * @return uint32_t 反馈离线判定时间, ms
     */
    inline uint32_t Get_Offline_Timeout_MS() const;

protected:
    UART_HandleTypeDef *UART_Handler = nullptr;

    // UART回调独占的协议解析器
    uint8_t Parser_State = 0U;
    uint8_t Rx_Frame_Buffer[32] = {0};
    uint8_t Rx_Frame_Index = 0U;
    uint8_t Rx_Command = 0U;
    uint8_t Rx_Payload_Length = 0U;
    uint32_t Last_Rx_Callback_Time_MS = 0U;

    // UART回调到1ms控制回调的反馈邮箱
    volatile int32_t Feedback_Encoder_A = 0;
    volatile int32_t Feedback_Encoder_B = 0;
    volatile int16_t Feedback_Speed_CPS_A = 0;
    volatile int16_t Feedback_Speed_CPS_B = 0;
    volatile uint8_t Feedback_Status = 0U;
    volatile uint8_t Feedback_Sequence = 0U;
    volatile bool Feedback_Pending = false;

    volatile uint8_t Remote_Status = 0U;
    volatile uint8_t Last_Feedback_Sequence = 0U;
    volatile bool Has_Valid_Feedback = false;

    volatile uint32_t Offline_Count_MS = 0U;
    volatile uint32_t Offline_Timeout_MS = MOTOR_TI_OFFLINE_TIMEOUT_MS;

    volatile bool Tx_Building = false;
    uint8_t Tx_Sequence = 0U;
    uint8_t Last_Rx_Sequence = 0U;
    bool Has_Previous_Rx_Sequence = false;

    volatile uint32_t Valid_Feedback_Count = 0U;
    volatile uint32_t CRC_Error_Count = 0U;
    volatile uint32_t Sequence_Error_Count = 0U;
    volatile uint32_t Pong_Count = 0U;
    volatile uint32_t Tx_Busy_Count = 0U;
    volatile uint32_t Tx_Error_Count = 0U;

private:
    // 重置协议解析器
    void Reset_Parser();

    // 逐字节解析协议
    void Parse_Byte(uint8_t Byte);

    // 处理通过CRC校验的完整帧
    void Accept_Frame(uint8_t Sequence,
                      uint8_t Command,
                      const uint8_t *Payload,
                      uint8_t Length);

    // 检查TI发送帧序号
    void Update_Rx_Sequence(uint8_t Sequence);

    // 取出并处理反馈邮箱
    bool Process_Feedback_Mailbox();

    // 更新A/B电机在线状态
    void Update_Online_Status();

    // 发送一帧TI协议数据
    bool Send_Frame(uint8_t Command, const uint8_t *Payload, uint8_t Length);

    // Modbus CRC16
    static uint16_t CRC16_Modbus(const uint8_t *Data, uint16_t Length);

    // PWM限幅
    static int16_t Clamp_PWM(int16_t PWM);

    // 写入16位大端有符号数
    static void Put_I16_BE(uint8_t *Buffer, int16_t Value);

    // 读取16位大端有符号数
    static int16_t Get_I16_BE(const uint8_t *Buffer);

    // 读取32位大端有符号数
    static int32_t Get_I32_BE(const uint8_t *Buffer);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_TI_Status 电机状态
 */
inline Enum_Motor_TI_Status Class_Motor_TI_Channel::Get_Status() const
{
    return (Motor_Status);
}

/**
 * @brief 获取电机通道
 *
 * @return Enum_Motor_TI_ID 电机通道
 */
inline Enum_Motor_TI_ID Class_Motor_TI_Channel::Get_Motor_ID() const
{
    return (Motor_ID);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_TI_Control_Method 电机控制方式
 */
inline Enum_Motor_TI_Control_Method Class_Motor_TI_Channel::Get_Control_Method() const
{
    return (Control_Method);
}

/**
 * @brief 获取电机逻辑方向
 *
 * @return Enum_Motor_TI_Direction 电机逻辑方向
 */
inline Enum_Motor_TI_Direction Class_Motor_TI_Channel::Get_Direction() const
{
    return (Direction);
}

/**
 * @brief 获取TI板上传的原始累计编码值
 *
 * @return int32_t 原始累计编码值
 */
inline int32_t Class_Motor_TI_Channel::Get_Now_Encoder() const
{
    return (Get_Rx_Data().Now_Encoder);
}

/**
 * @brief 获取H723侧累计编码值
 *
 * @return int64_t 累计编码值
 */
inline int64_t Class_Motor_TI_Channel::Get_Total_Encoder() const
{
    return (Get_Rx_Data().Total_Encoder);
}

/**
 * @brief 获取当前输出轴角度
 *
 * @return float 当前角度, rad
 */
inline float Class_Motor_TI_Channel::Get_Now_Angle() const
{
    return (Get_Rx_Data().Now_Angle);
}

/**
 * @brief 获取当前输出轴角速度
 *
 * @return float 当前角速度, rad/s
 */
inline float Class_Motor_TI_Channel::Get_Now_Omega() const
{
    return (Get_Rx_Data().Now_Omega);
}

/**
 * @brief 获取目标PWM
 *
 * @return float 目标PWM
 */
inline float Class_Motor_TI_Channel::Get_Target_PWM() const
{
    return (Target_PWM);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度, rad/s
 */
inline float Class_Motor_TI_Channel::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标角度
 *
 * @return float 目标角度, rad
 */
inline float Class_Motor_TI_Channel::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取PWM前馈值
 *
 * @return float PWM前馈值
 */
inline float Class_Motor_TI_Channel::Get_Feedforward_PWM() const
{
    return (Feedforward_PWM);
}

/**
 * @brief 获取角速度前馈值
 *
 * @return float 角速度前馈值, rad/s
 */
inline float Class_Motor_TI_Channel::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取当前控制输出
 *
 * @return float 当前PWM输出
 */
inline float Class_Motor_TI_Channel::Get_Out() const
{
    return (Out);
}

/**
 * @brief 获取输出轴每圈软件计数
 *
 * @return float 输出轴每圈软件计数
 */
inline float Class_Motor_TI_Channel::Get_Encoder_Num_Per_Round() const
{
    return (Encoder_Num_Per_Round);
}

/**
 * @brief 获取PWM输出限幅
 *
 * @return float PWM输出限幅
 */
inline float Class_Motor_TI_Channel::Get_PWM_Limit() const
{
    return (PWM_Limit);
}

/**
 * @brief 获取每毫秒PWM最大变化量
 *
 * @return float 每毫秒PWM最大变化量
 */
inline float Class_Motor_TI_Channel::Get_PWM_Change_Max_Per_MS() const
{
    return (PWM_Change_Max_Per_MS);
}

/**
 * @brief 获取控制使能状态
 *
 * @return true 控制使能
 * @return false 控制失能
 */
inline bool Class_Motor_TI_Channel::Get_Control_Enable() const
{
    return (Control_Enable);
}

/**
 * @brief 设置目标PWM
 *
 * @param __Target_PWM 目标PWM
 */
inline void Class_Motor_TI_Channel::Set_Target_PWM(float __Target_PWM)
{
    Target_PWM = __Target_PWM;
}

/**
 * @brief 设置目标角速度
 *
 * @param __Target_Omega 目标角速度, rad/s
 */
inline void Class_Motor_TI_Channel::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设置目标角度
 *
 * @param __Target_Angle 目标角度, rad
 */
inline void Class_Motor_TI_Channel::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设置PWM前馈值
 *
 * @param __Feedforward_PWM PWM前馈值
 */
inline void Class_Motor_TI_Channel::Set_Feedforward_PWM(float __Feedforward_PWM)
{
    Feedforward_PWM = __Feedforward_PWM;
}

/**
 * @brief 设置角速度前馈值
 *
 * @param __Feedforward_Omega 角速度前馈值, rad/s
 */
inline void Class_Motor_TI_Channel::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 获取TI驱动板状态字节
 *
 * @return uint8_t 状态字节
 */
inline uint8_t Class_Motor_TI::Get_Remote_Status() const
{
    return (Remote_Status);
}

/**
 * @brief 获取最后一帧反馈序号
 *
 * @return uint8_t 最后一帧反馈序号
 */
inline uint8_t Class_Motor_TI::Get_Last_Feedback_Sequence() const
{
    return (Last_Feedback_Sequence);
}

/**
 * @brief 获取反馈离线计时
 *
 * @return uint32_t 离线计时, ms
 */
inline uint32_t Class_Motor_TI::Get_Offline_Count_MS() const
{
    return (Offline_Count_MS);
}

/**
 * @brief 设置反馈离线判定时间
 *
 * @param __Offline_Timeout_MS 反馈离线判定时间, ms
 */
inline void Class_Motor_TI::Set_Offline_Timeout_MS(uint32_t __Offline_Timeout_MS)
{
    Offline_Timeout_MS = __Offline_Timeout_MS;
}

/**
 * @brief 获取反馈离线判定时间
 *
 * @return uint32_t 反馈离线判定时间, ms
 */
inline uint32_t Class_Motor_TI::Get_Offline_Timeout_MS() const
{
    return (Offline_Timeout_MS);
}

#endif /* DVC_MOTOR_TI_H */

/*----------------------------------------------------------------------------*/
