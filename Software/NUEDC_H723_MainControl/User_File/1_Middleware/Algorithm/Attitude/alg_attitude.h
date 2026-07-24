/**
 * @file alg_attitude.h
 * @author WangFonzhuo
 * @brief 基于Mahony六轴融合的四元数姿态解算
 * @version 1.0
 * @date 2026-07-23 27赛季
 * @note 输入角速度单位为rad/s, 加速度单位为m/s^2.
 *       六轴IMU无法观测绝对航向, Yaw会随时间缓慢漂移.
 */

#ifndef ALG_ATTITUDE_H
#define ALG_ATTITUDE_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define ATTITUDE_MAHONY_PI                    (3.14159265358979323846f)
#define ATTITUDE_MAHONY_RAD_TO_DEGREE         \
    (180.0f / ATTITUDE_MAHONY_PI)
#define ATTITUDE_MAHONY_STANDARD_GRAVITY      (9.80665f)
#define ATTITUDE_MAHONY_MIN_NORM_SQUARED      (1.0e-12f)
#define ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS  (0.2f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Mahony六轴姿态解算器
 *
 * @note 坐标系必须为右手系. 欧拉角定义为:
 *       Roll绕+X, Pitch绕+Y, Yaw绕+Z, 旋转正方向符合右手定则.
 * @note 四元数表示机体系到参考系的旋转, 顺序为W、X、Y、Z.
 */
class Class_Attitude_Mahony
{
public:
    /**
     * @brief 初始化姿态解算器
     *
     * @param __Sample_Period_s 固定更新周期, 单位s
     * @param __K_P 加速度重力方向比例修正增益
     * @param __K_I 陀螺仪残余零偏积分修正增益
     * @param __Accel_Rejection_Ratio 加速度模长相对1g的拒绝阈值
     */
    void Init(float __Sample_Period_s = 0.001f,
              float __K_P = 2.0f,
              float __K_I = 0.0f,
              float __Accel_Rejection_Ratio = 0.30f);

    /**
     * @brief 清空姿态并回到未初始化状态
     */
    void Reset();

    /**
     * @brief 用当前加速度方向初始化Roll和Pitch, Yaw置0
     *
     * @return bool 加速度是否合法
     */
    bool Initialize_From_Accelerometer(float __Accel_X,
                                       float __Accel_Y,
                                       float __Accel_Z);

    /**
     * @brief 执行一次Mahony六轴融合
     *
     * @param __Gyro_X X轴角速度, rad/s
     * @param __Gyro_Y Y轴角速度, rad/s
     * @param __Gyro_Z Z轴角速度, rad/s
     * @param __Accel_X X轴加速度, m/s^2
     * @param __Accel_Y Y轴加速度, m/s^2
     * @param __Accel_Z Z轴加速度, m/s^2
     * @return bool 本次是否成功更新
     */
    bool Update(float __Gyro_X,
                float __Gyro_Y,
                float __Gyro_Z,
                float __Accel_X,
                float __Accel_Y,
                float __Accel_Z);

    /**
     * @brief 使用实际采样间隔执行一次Mahony六轴融合
     *
     * @param __D_T_s 本次陀螺仪样本间隔, 单位s
     * @param __Accel_Update_Flag 本次是否有新的加速度计观测
     * @return bool 本次是否成功更新
     */
    bool Update_With_D_T(float __Gyro_X,
                         float __Gyro_Y,
                         float __Gyro_Z,
                         float __Accel_X,
                         float __Accel_Y,
                         float __Accel_Z,
                         float __D_T_s,
                         bool __Accel_Update_Flag);

    /**
     * @brief 保持当前Roll和Pitch并将Yaw重新置0
     */
    void Reset_Yaw();

    void Set_Gains(float __K_P, float __K_I);
    void Set_Sample_Period(float __Sample_Period_s);
    void Set_Accel_Rejection_Ratio(float __Accel_Rejection_Ratio);

    inline bool Get_Initialized_Flag() const;
    inline bool Get_Accel_Correction_Valid_Flag() const;
    inline uint32_t Get_Update_Count() const;
    inline uint32_t Get_Accel_Rejection_Count() const;
    inline float Get_Accel_Correction_Weight() const;
    inline float Get_Quaternion_W() const;
    inline float Get_Quaternion_X() const;
    inline float Get_Quaternion_Y() const;
    inline float Get_Quaternion_Z() const;
    inline float Get_Roll() const;
    inline float Get_Pitch() const;
    inline float Get_Yaw() const;
    inline float Get_Roll_Degree() const;
    inline float Get_Pitch_Degree() const;
    inline float Get_Yaw_Degree() const;

protected:
    float Sample_Period_s = 0.001f;
    float K_P = 2.0f;
    float K_I = 0.0f;
    float Accel_Rejection_Ratio = 0.30f;

    float Quaternion_W = 1.0f;
    float Quaternion_X = 0.0f;
    float Quaternion_Y = 0.0f;
    float Quaternion_Z = 0.0f;

    float Integral_Feedback_X = 0.0f;
    float Integral_Feedback_Y = 0.0f;
    float Integral_Feedback_Z = 0.0f;

    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;

    bool Initialized_Flag = false;
    bool Accel_Correction_Valid_Flag = false;
    uint32_t Update_Count = 0U;
    uint32_t Accel_Rejection_Count = 0U;
    float Accel_Correction_Weight = 0.0f;

    void Set_Quaternion_From_Euler(float __Roll,
                                   float __Pitch,
                                   float __Yaw);
    bool Normalize_Quaternion();
    void Update_Euler();
    static float Limit(float __Value,
                       float __Minimum,
                       float __Maximum);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

inline bool Class_Attitude_Mahony::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline bool Class_Attitude_Mahony::Get_Accel_Correction_Valid_Flag() const
{
    return Accel_Correction_Valid_Flag;
}

inline uint32_t Class_Attitude_Mahony::Get_Update_Count() const
{
    return Update_Count;
}

inline uint32_t Class_Attitude_Mahony::Get_Accel_Rejection_Count() const
{
    return Accel_Rejection_Count;
}

inline float Class_Attitude_Mahony::Get_Accel_Correction_Weight() const
{
    return Accel_Correction_Weight;
}

inline float Class_Attitude_Mahony::Get_Quaternion_W() const
{
    return Quaternion_W;
}

inline float Class_Attitude_Mahony::Get_Quaternion_X() const
{
    return Quaternion_X;
}

inline float Class_Attitude_Mahony::Get_Quaternion_Y() const
{
    return Quaternion_Y;
}

inline float Class_Attitude_Mahony::Get_Quaternion_Z() const
{
    return Quaternion_Z;
}

inline float Class_Attitude_Mahony::Get_Roll() const
{
    return Roll;
}

inline float Class_Attitude_Mahony::Get_Pitch() const
{
    return Pitch;
}

inline float Class_Attitude_Mahony::Get_Yaw() const
{
    return Yaw;
}

inline float Class_Attitude_Mahony::Get_Roll_Degree() const
{
    return Roll * ATTITUDE_MAHONY_RAD_TO_DEGREE;
}

inline float Class_Attitude_Mahony::Get_Pitch_Degree() const
{
    return Pitch * ATTITUDE_MAHONY_RAD_TO_DEGREE;
}

inline float Class_Attitude_Mahony::Get_Yaw_Degree() const
{
    return Yaw * ATTITUDE_MAHONY_RAD_TO_DEGREE;
}

#endif /* ALG_ATTITUDE_H */

/*----------------------------------------------------------------------------*/
