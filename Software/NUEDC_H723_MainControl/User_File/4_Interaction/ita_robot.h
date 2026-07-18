/**
 * @file ita_robot.h
 * @author WangFonzhuo
 * @brief 整车人机交互与操作逻辑
 * @version 1.0
 * @date 2026-07-18
 * @note 当前版本仅实现DR16对两轮差速底盘的手动控制
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
 *
 * @note 当前操作映射：
 *       1. DR16在线且右拨杆稳定处于UP时，底盘使能；
 *       2. 右拨杆处于MIDDLE/DOWN或DR16掉线时，底盘失能；
 *       3. 左摇杆Y轴控制底盘前进/后退；
 *       4. 左摇杆X轴控制底盘旋转；
 *       5. 右摇杆保留给后续云台控制。
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
     * @brief 1ms整车操作逻辑回调函数
     *
     * @note 应在底盘解算与控制计算之前调用。
     */
    void TIM_1ms_Control_PeriodElapsedCallback();

    inline bool Get_Chassis_Control_Enable() const;
    inline float Get_Chassis_Max_Velocity_X() const;
    inline float Get_Chassis_Max_Omega() const;
    inline float Get_DR16_Dead_Zone() const;

    void Set_Chassis_Max_Velocity_X(float __Max_Velocity_X);
    void Set_Chassis_Max_Omega(float __Max_Omega);
    void Set_DR16_Dead_Zone(float __Dead_Zone);

protected:
    Class_DR16 *DR16 = nullptr;
    Class_Chassis *Chassis = nullptr;

    bool Chassis_Control_Enable = false;

    float Chassis_Max_Velocity_X =
        ROBOT_CHASSIS_MAX_VELOCITY_X_DEFAULT;

    float Chassis_Max_Omega =
        ROBOT_CHASSIS_MAX_OMEGA_DEFAULT;

    float DR16_Dead_Zone =
        ROBOT_DR16_DEAD_ZONE_DEFAULT;

private:
    /**
     * @brief 对摇杆输入应用死区并重新映射到[-1, 1]
     */
    float Apply_Dead_Zone(float Input) const;

    /**
     * @brief 安全失能底盘
     */
    void Disable_Chassis();
};

/* Exported variables --------------------------------------------------------*/

extern Class_Robot Robot;

/* Exported function definitions ---------------------------------------------*/

inline bool Class_Robot::Get_Chassis_Control_Enable() const
{
    return (Chassis_Control_Enable);
}

inline float Class_Robot::Get_Chassis_Max_Velocity_X() const
{
    return (Chassis_Max_Velocity_X);
}

inline float Class_Robot::Get_Chassis_Max_Omega() const
{
    return (Chassis_Max_Omega);
}

inline float Class_Robot::Get_DR16_Dead_Zone() const
{
    return (DR16_Dead_Zone);
}

#endif /* ITA_ROBOT_H */

/*----------------------------------------------------------------------------*/
