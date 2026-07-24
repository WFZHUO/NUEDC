/**
 * @file ita_robot.h
 * @author WangFangzhuo
 * @brief 整车人机交互与操作逻辑
 * @version 1.0
 * @date 2026-07-18
 */

#ifndef ITA_ROBOT_H
#define ITA_ROBOT_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "crt_chassis.h"

/* Exported macros -----------------------------------------------------------*/

// 底盘最大前进/后退线速度，单位m/s
#define ROBOT_CHASSIS_MAX_VELOCITY_X_DEFAULT 0.30f

// 底盘最大旋转角速度，单位rad/s
#define ROBOT_CHASSIS_MAX_OMEGA_DEFAULT 2.00f

// DR16摇杆死区
#define ROBOT_DR16_DEAD_ZONE_DEFAULT 0.05f

// 左摇杆Y轴与底盘X轴线速度之间的方向映射
#define ROBOT_CHASSIS_VELOCITY_X_SIGN 1.0f

// 左摇杆X向右为正，而底盘逆时针Omega为正，因此默认取反
#define ROBOT_CHASSIS_OMEGA_SIGN (-1.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 整车人机交互控制类
 */
class Class_Robot
{
public:
    /**
     * @brief 初始化整车操作逻辑
     *
     * @param __DR16 DR16遥控器对象
     * @param __Chassis 两轮差速底盘对象
     */
    void Init(Class_DR16 *__DR16, Class_Chassis *__Chassis);

    /**
     * @brief 获得底盘控制使能状态
     *
     * @return bool 底盘控制使能状态
     */
    inline bool Get_Chassis_Control_Enable() const;

    /**
     * @brief 获得底盘最大X轴速度
     *
     * @return float 最大X轴速度
     */
    inline float Get_Chassis_Max_Velocity_X() const;

    /**
     * @brief 获得底盘最大旋转角速度
     *
     * @return float 最大旋转角速度
     */
    inline float Get_Chassis_Max_Omega() const;

    /**
     * @brief 获得DR16摇杆死区
     *
     * @return float DR16摇杆死区
     */
    inline float Get_DR16_Dead_Zone() const;

    /**
     * @brief 设置底盘最大X轴速度
     *
     * @param __Max_Velocity_X 最大X轴速度
     */
    inline void Set_Chassis_Max_Velocity_X(float __Max_Velocity_X);

    /**
     * @brief 设置底盘最大旋转角速度
     *
     * @param __Max_Omega 最大旋转角速度
     */
    inline void Set_Chassis_Max_Omega(float __Max_Omega);

    /**
     * @brief 设置DR16摇杆死区
     *
     * @param __Dead_Zone DR16摇杆死区
     */
    inline void Set_DR16_Dead_Zone(float __Dead_Zone);

    /**
     * @brief 1ms整车操作逻辑回调函数
     */
    void TIM_1ms_Control_PeriodElapsedCallback();

protected:
    // 绑定对象
    Class_DR16 *DR16 = nullptr;
    Class_Chassis *Chassis = nullptr;

    // 底盘控制使能状态
    bool Chassis_Control_Enable = false;

    // 底盘最大X轴速度，单位m/s
    float Chassis_Max_Velocity_X = ROBOT_CHASSIS_MAX_VELOCITY_X_DEFAULT;

    // 底盘最大旋转角速度，单位rad/s
    float Chassis_Max_Omega = ROBOT_CHASSIS_MAX_OMEGA_DEFAULT;

    // DR16摇杆死区
    float DR16_Dead_Zone = ROBOT_DR16_DEAD_ZONE_DEFAULT;

    // 对摇杆输入应用死区并重新映射到[-1, 1]
    float Apply_Dead_Zone(float Input) const;

    // 禁用底盘控制
    void Disable_Chassis();
};

/* Exported variables --------------------------------------------------------*/

// 整车人机交互控制对象
extern Class_Robot Robot;

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 获得底盘控制使能状态
 *
 * @return bool 底盘控制使能状态
 */
inline bool Class_Robot::Get_Chassis_Control_Enable() const
{
    return (Chassis_Control_Enable);
}

/**
 * @brief 获得底盘最大X轴速度
 *
 * @return float 最大X轴速度
 */
inline float Class_Robot::Get_Chassis_Max_Velocity_X() const
{
    return (Chassis_Max_Velocity_X);
}

/**
 * @brief 获得底盘最大旋转角速度
 *
 * @return float 最大旋转角速度
 */
inline float Class_Robot::Get_Chassis_Max_Omega() const
{
    return (Chassis_Max_Omega);
}

/**
 * @brief 获得DR16摇杆死区
 *
 * @return float DR16摇杆死区
 */
inline float Class_Robot::Get_DR16_Dead_Zone() const
{
    return (DR16_Dead_Zone);
}

/**
 * @brief 设置底盘最大前进/后退线速度
 *
 * @param __Max_Velocity_X 最大X轴速度
 */
inline void Class_Robot::Set_Chassis_Max_Velocity_X(float __Max_Velocity_X)
{
    if (__Max_Velocity_X > 0.0f)
    {
        Chassis_Max_Velocity_X = __Max_Velocity_X;
    }
}

/**
 * @brief 设置底盘最大旋转角速度
 *
 * @param __Max_Omega 最大旋转角速度
 */
inline void Class_Robot::Set_Chassis_Max_Omega(float __Max_Omega)
{
    if (__Max_Omega > 0.0f)
    {
        Chassis_Max_Omega = __Max_Omega;
    }
}

/**
 * @brief 设置DR16摇杆死区
 *
 * @param __Dead_Zone DR16摇杆死区
 */
inline void Class_Robot::Set_DR16_Dead_Zone(float __Dead_Zone)
{
    if (__Dead_Zone >= 0.0f && __Dead_Zone < 1.0f)
    {
        DR16_Dead_Zone = __Dead_Zone;
    }
}

#endif /* ITA_ROBOT_H */

/*----------------------------------------------------------------------------*/
