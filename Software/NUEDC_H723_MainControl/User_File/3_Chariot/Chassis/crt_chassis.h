/**
 * @file crt_chassis.h
 * @author WangFonzhuo
 * @brief 两轮差速底盘运动解算
 * @version 1.0
 * @date 2026-07-18
 * @note 底盘坐标系：X轴指向车体前方，Z轴指向车体上方；
 *       底盘角速度Omega正方向为从车体上方向下看逆时针。
 * @note TI驱动板通道与车轮对应关系：Motor_A为右轮，Motor_B为左轮。
 * @note 单轮机械角速度正方向统一定义为：从该电机输出轴侧向电机内部看，
 *       逆时针为正，顺时针为负。当前TI板反馈数据已经符合该定义。
 * @note Enum_Motor_TI_Direction只用于修正电机PWM输出极性，不改变编码器角度
 *       与角速度反馈的正负方向。当前实车A、B两路均需使用
 *       Motor_TI_Direction_REVERSE，才能使电机机械正目标角速度对应输出轴
 *       逆时针旋转，并与电机机械正反馈角速度保持一致。
 * @note 左右电机为镜像安装，因此“两个输出轴均逆时针为正”不等价于
 *       “两个车轮均驱动车体向前”。本底盘额外定义车轮逻辑正方向：
 *       驱动车体沿X轴向前为正。当前安装下左轮机械正方向对应车体向前，
 *       右轮机械负方向对应车体向前。
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_ti.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// 默认轮子半径，单位m
#define CHASSIS_DEFAULT_WHEEL_RADIUS 0.0235f

// 默认半轮距：左右驱动轮轮心之间横向距离的一半，单位m
// 底盘坐标系原点定义在左右驱动轮轮心连线的中点
#define CHASSIS_DEFAULT_HALF_TRACK 0.0635f

// 默认不启用单轮线速度限幅，实车测速后通过Set_Wheel_Linear_Speed_Limit设置
#define CHASSIS_DEFAULT_WHEEL_LINEAR_SPEED_LIMIT 0.0f

// 当前实车左轮默认PWM输出方向
#define CHASSIS_DEFAULT_LEFT_MOTOR_DIRECTION Motor_TI_Direction_REVERSE

// 当前实车右轮默认PWM输出方向
#define CHASSIS_DEFAULT_RIGHT_MOTOR_DIRECTION Motor_TI_Direction_REVERSE

/*
 * 电机机械角速度与车轮逻辑角速度之间的符号映射。
 *
 * 电机机械角速度正方向：
 * 从各自输出轴侧向电机内部看，逆时针为正。
 *
 * 车轮逻辑角速度正方向：
 * 该车轮驱动车体沿底盘X轴向前为正。
 *
 * 当前左右电机镜像安装：
 * 左轮B路机械正方向可驱动车体向前，因此符号为+1；
 * 右轮A路机械负方向才可驱动车体向前，因此符号为-1。
 *
 * 该符号同时用于：
 * 1. 将TI机械反馈转换为底盘逻辑轮速；
 * 2. 将底盘逻辑轮速目标转换为TI机械角速度目标。
 */
#define CHASSIS_LEFT_WHEEL_FORWARD_SIGN  1.0f
#define CHASSIS_RIGHT_WHEEL_FORWARD_SIGN (-1.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘控制状态
 */
enum Enum_Chassis_Control_State
{
    Chassis_Control_State_DISABLE = 0,
    Chassis_Control_State_NORMAL,
};

/**
 * @brief 两轮差速底盘类
 *
 * @note 该类将上层给出的底盘线速度与角速度目标逆解为左右轮目标角速度，
 *       再通过Class_Motor_TI完成双路电机速度闭环和PWM命令发送。
 * @note Motor_TI.Motor_A固定对应右轮，Motor_TI.Motor_B固定对应左轮。
 * @note 类内部会处理镜像安装导致的左右轮机械角速度符号差异；
 *       对上层而言，左右轮逻辑正速度均表示驱动车体向前。
 */
class Class_Chassis
{
public:
    // TI双路有刷电机驱动板对象，Motor_A为右轮，Motor_B为左轮
    Class_Motor_TI Motor_TI;

    /**
     * @brief 初始化底盘
     *
     * @param huart 与TI驱动板连接的UART
     * @param __Wheel_Radius 轮子半径，单位m
     * @param __Chassis_Half_Track 左右驱动轮轮心之间横向距离的一半，单位m
     * @param __Wheel_Linear_Speed_Limit 单轮线速度限幅，单位m/s，0表示不限制
     * @param __Encoder_Num_Per_Round 输出轴每圈软件累计计数
     * @param __Direction_Left 左轮PWM输出方向，默认REVERSE
     * @param __Direction_Right 右轮PWM输出方向，默认REVERSE
     *
     * @note 方向参数只影响下发到TI板的PWM正负，不改变编码器反馈方向。
     * @note 当前实车已经验证左右轮均应使用Motor_TI_Direction_REVERSE。
     * @note 参数按左轮、右轮排列；传入Class_Motor_TI时会根据
     *       Motor_A为右轮、Motor_B为左轮的映射关系自动交换为A、B顺序。
     */
    void Init(UART_HandleTypeDef *huart,
              float __Wheel_Radius = CHASSIS_DEFAULT_WHEEL_RADIUS,
              float __Chassis_Half_Track = CHASSIS_DEFAULT_HALF_TRACK,
              float __Wheel_Linear_Speed_Limit = CHASSIS_DEFAULT_WHEEL_LINEAR_SPEED_LIMIT,
              float __Encoder_Num_Per_Round = MOTOR_TI_ENCODER_NUM_PER_ROUND,
              Enum_Motor_TI_Direction __Direction_Left = CHASSIS_DEFAULT_LEFT_MOTOR_DIRECTION,
              Enum_Motor_TI_Direction __Direction_Right = CHASSIS_DEFAULT_RIGHT_MOTOR_DIRECTION);

    /**
     * @brief UART通信接收回调函数
     *
     * @param Rx_Data 接收数据缓冲区
     * @param Length 接收数据长度
     *
     * @note 应在与TI驱动板连接的UART接收回调中调用。
     */
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    /**
     * @brief 1ms底盘解算与控制计算回调函数
     *
     * @note 依次完成差速底盘运动学逆解、电机目标写入、TI反馈处理、
     *       双轮速度闭环计算以及底盘运动状态正解。
     */
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    /**
     * @brief 1ms底盘控制命令发送回调函数
     *
     * @return true 发送启动成功
     * @return false UART忙或发送失败
     *
     * @note 该函数将双轮控制结果通过TI通信协议发送到驱动板。
     */
    bool TIM_1ms_Write_PeriodElapsedCallback();

    /**
     * @brief 获取底盘控制状态
     *
     * @return Enum_Chassis_Control_State 底盘控制状态
     */
    inline Enum_Chassis_Control_State Get_Control_State() const;

    /**
     * @brief 获取当前底盘X轴线速度
     *
     * @return float 当前底盘X轴线速度，单位m/s
     */
    inline float Get_Now_Velocity_X() const;

    /**
     * @brief 获取当前底盘角速度
     *
     * @return float 当前底盘角速度，单位rad/s
     */
    inline float Get_Now_Omega() const;

    /**
     * @brief 获取上层设置的底盘X轴目标线速度
     *
     * @return float 底盘X轴目标线速度，单位m/s
     */
    inline float Get_Target_Velocity_X() const;

    /**
     * @brief 获取上层设置的底盘目标角速度
     *
     * @return float 底盘目标角速度，单位rad/s
     */
    inline float Get_Target_Omega() const;

    /**
     * @brief 获取经过单轮速度限幅后的底盘X轴目标线速度
     *
     * @return float 限幅后的底盘X轴目标线速度，单位m/s
     */
    inline float Get_Resolved_Target_Velocity_X() const;

    /**
     * @brief 获取经过单轮速度限幅后的底盘目标角速度
     *
     * @return float 限幅后的底盘目标角速度，单位rad/s
     */
    inline float Get_Resolved_Target_Omega() const;

    /**
     * @brief 获取左轮目标角速度
     *
     * @return float 左轮目标角速度，单位rad/s
     */
    inline float Get_Target_Left_Wheel_Omega() const;

    /**
     * @brief 获取右轮目标角速度
     *
     * @return float 右轮目标角速度，单位rad/s
     */
    inline float Get_Target_Right_Wheel_Omega() const;

    /**
     * @brief 获取轮子半径
     *
     * @return float 轮子半径，单位m
     */
    inline float Get_Wheel_Radius() const;

    /**
     * @brief 获取底盘半轮距
     *
     * @return float 左右驱动轮轮心横向距离的一半，单位m
     */
    inline float Get_Chassis_Half_Track() const;

    /**
     * @brief 获取单轮线速度限幅
     *
     * @return float 单轮线速度限幅，单位m/s；0表示不限制
     */
    inline float Get_Wheel_Linear_Speed_Limit() const;

    /**
     * @brief 设置底盘控制状态
     *
     * @param __Control_State 底盘控制状态
     *
     * @note 切换为DISABLE时会立即清空整车目标和双轮控制器。
     */
    void Set_Control_State(Enum_Chassis_Control_State __Control_State);

    /**
     * @brief 设置底盘X轴目标线速度
     *
     * @param __Target_Velocity_X 底盘X轴目标线速度，单位m/s
     */
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    /**
     * @brief 设置底盘目标角速度
     *
     * @param __Target_Omega 底盘目标角速度，单位rad/s
     */
    inline void Set_Target_Omega(float __Target_Omega);

    /**
     * @brief 设置单轮线速度限幅
     *
     * @param __Wheel_Linear_Speed_Limit 单轮线速度限幅，单位m/s
     *
     * @note 参数小于等于0时关闭单轮线速度限幅。
     */
    void Set_Wheel_Linear_Speed_Limit(float __Wheel_Linear_Speed_Limit);

    /**
     * @brief 清空底盘运动目标与双轮控制器
     *
     * @note 不清除左右轮累计编码位置，也不修改底盘结构参数。
     */
    void Clear_Control();

protected:
    // 底盘结构参数
    float Wheel_Radius = CHASSIS_DEFAULT_WHEEL_RADIUS;
    float Chassis_Half_Track = CHASSIS_DEFAULT_HALF_TRACK;
    float Wheel_Linear_Speed_Limit = CHASSIS_DEFAULT_WHEEL_LINEAR_SPEED_LIMIT;

    // 当前底盘运动状态
    float Now_Velocity_X = 0.0f;
    float Now_Omega = 0.0f;

    // 上层给出的底盘运动目标
    float Target_Velocity_X = 0.0f;
    float Target_Omega = 0.0f;

    // 经过单轮线速度限幅后的实际解算目标
    float Resolved_Target_Velocity_X = 0.0f;
    float Resolved_Target_Omega = 0.0f;

    // 左右轮目标角速度，单位rad/s
    float Target_Left_Wheel_Omega = 0.0f;
    float Target_Right_Wheel_Omega = 0.0f;

    // 底盘控制状态，默认失能
    Enum_Chassis_Control_State Control_State = Chassis_Control_State_DISABLE;

private:
    /**
     * @brief 根据左右轮反馈角速度正解底盘速度
     */
    void Self_Resolution();

    /**
     * @brief 根据底盘目标速度逆解左右轮目标角速度
     */
    void Kinematics_Inverse_Resolution();

    /**
     * @brief 将左右轮目标角速度输出到对应电机对象
     */
    void Output_To_Motor();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 获取底盘控制状态
 *
 * @return Enum_Chassis_Control_State 底盘控制状态
 */
inline Enum_Chassis_Control_State Class_Chassis::Get_Control_State() const
{
    return (Control_State);
}

/**
 * @brief 获取当前底盘X轴线速度
 *
 * @return float 当前底盘X轴线速度，单位m/s
 */
inline float Class_Chassis::Get_Now_Velocity_X() const
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前底盘角速度
 *
 * @return float 当前底盘角速度，单位rad/s
 */
inline float Class_Chassis::Get_Now_Omega() const
{
    return (Now_Omega);
}

/**
 * @brief 获取上层设置的底盘X轴目标线速度
 *
 * @return float 底盘X轴目标线速度，单位m/s
 */
inline float Class_Chassis::Get_Target_Velocity_X() const
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取上层设置的底盘目标角速度
 *
 * @return float 底盘目标角速度，单位rad/s
 */
inline float Class_Chassis::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取经过单轮速度限幅后的底盘X轴目标线速度
 *
 * @return float 限幅后的底盘X轴目标线速度，单位m/s
 */
inline float Class_Chassis::Get_Resolved_Target_Velocity_X() const
{
    return (Resolved_Target_Velocity_X);
}

/**
 * @brief 获取经过单轮速度限幅后的底盘目标角速度
 *
 * @return float 限幅后的底盘目标角速度，单位rad/s
 */
inline float Class_Chassis::Get_Resolved_Target_Omega() const
{
    return (Resolved_Target_Omega);
}

/**
 * @brief 获取左轮目标角速度
 *
 * @return float 左轮目标角速度，单位rad/s
 */
inline float Class_Chassis::Get_Target_Left_Wheel_Omega() const
{
    return (Target_Left_Wheel_Omega);
}

/**
 * @brief 获取右轮目标角速度
 *
 * @return float 右轮目标角速度，单位rad/s
 */
inline float Class_Chassis::Get_Target_Right_Wheel_Omega() const
{
    return (Target_Right_Wheel_Omega);
}

/**
 * @brief 获取轮子半径
 *
 * @return float 轮子半径，单位m
 */
inline float Class_Chassis::Get_Wheel_Radius() const
{
    return (Wheel_Radius);
}

/**
 * @brief 获取底盘半轮距
 *
 * @return float 左右驱动轮轮心横向距离的一半，单位m
 */
inline float Class_Chassis::Get_Chassis_Half_Track() const
{
    return (Chassis_Half_Track);
}

/**
 * @brief 获取单轮线速度限幅
 *
 * @return float 单轮线速度限幅，单位m/s；0表示不限制
 */
inline float Class_Chassis::Get_Wheel_Linear_Speed_Limit() const
{
    return (Wheel_Linear_Speed_Limit);
}

/**
 * @brief 设置底盘X轴目标线速度
 *
 * @param __Target_Velocity_X 底盘X轴目标线速度，单位m/s
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设置底盘目标角速度
 *
 * @param __Target_Omega 底盘目标角速度，单位rad/s
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

#endif /* CRT_CHASSIS_H */

/*----------------------------------------------------------------------------*/
