/**
 * @file alg_attitude.cpp
 * @author WangFonzhuo
 * @brief 基于Mahony六轴融合的四元数姿态解算
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_attitude.h"

#include <cmath>

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 将数值限制在指定范围
 */
float Class_Attitude_Mahony::Limit(float __Value,
                                    float __Minimum,
                                    float __Maximum)
{
    if (__Value < __Minimum)
    {
        return __Minimum;
    }

    if (__Value > __Maximum)
    {
        return __Maximum;
    }

    return __Value;
}

/**
 * @brief 初始化姿态解算器
 */
void Class_Attitude_Mahony::Init(float __Sample_Period_s,
                                  float __K_P,
                                  float __K_I,
                                  float __Accel_Rejection_Ratio)
{
    Set_Sample_Period(__Sample_Period_s);
    Set_Gains(__K_P, __K_I);
    Set_Accel_Rejection_Ratio(__Accel_Rejection_Ratio);
    Reset();
}

/**
 * @brief 清空姿态并回到未初始化状态
 */
void Class_Attitude_Mahony::Reset()
{
    Quaternion_W = 1.0f;
    Quaternion_X = 0.0f;
    Quaternion_Y = 0.0f;
    Quaternion_Z = 0.0f;

    Integral_Feedback_X = 0.0f;
    Integral_Feedback_Y = 0.0f;
    Integral_Feedback_Z = 0.0f;

    Roll = 0.0f;
    Pitch = 0.0f;
    Yaw = 0.0f;

    Initialized_Flag = false;
    Accel_Correction_Valid_Flag = false;
    Update_Count = 0U;
    Accel_Rejection_Count = 0U;
    Accel_Correction_Weight = 0.0f;
}

/**
 * @brief 配置Mahony比例和积分增益
 */
void Class_Attitude_Mahony::Set_Gains(float __K_P, float __K_I)
{
    K_P = (__K_P >= 0.0f) ? __K_P : 0.0f;
    K_I = (__K_I >= 0.0f) ? __K_I : 0.0f;

    if (K_I <= 0.0f)
    {
        Integral_Feedback_X = 0.0f;
        Integral_Feedback_Y = 0.0f;
        Integral_Feedback_Z = 0.0f;
    }
}

/**
 * @brief 配置固定更新周期
 */
void Class_Attitude_Mahony::Set_Sample_Period(float __Sample_Period_s)
{
    if (__Sample_Period_s > 0.0f)
    {
        Sample_Period_s = __Sample_Period_s;
    }
}

/**
 * @brief 配置线加速度拒绝阈值
 */
void Class_Attitude_Mahony::Set_Accel_Rejection_Ratio(
    float __Accel_Rejection_Ratio)
{
    Accel_Rejection_Ratio =
        (__Accel_Rejection_Ratio >= 0.0f) ?
        __Accel_Rejection_Ratio : 0.0f;
}

/**
 * @brief 根据ZYX欧拉角生成四元数
 */
void Class_Attitude_Mahony::Set_Quaternion_From_Euler(float __Roll,
                                                       float __Pitch,
                                                       float __Yaw)
{
    const float half_roll = 0.5f * __Roll;
    const float half_pitch = 0.5f * __Pitch;
    const float half_yaw = 0.5f * __Yaw;

    const float cos_roll = cosf(half_roll);
    const float sin_roll = sinf(half_roll);
    const float cos_pitch = cosf(half_pitch);
    const float sin_pitch = sinf(half_pitch);
    const float cos_yaw = cosf(half_yaw);
    const float sin_yaw = sinf(half_yaw);

    Quaternion_W =
        cos_roll * cos_pitch * cos_yaw +
        sin_roll * sin_pitch * sin_yaw;
    Quaternion_X =
        sin_roll * cos_pitch * cos_yaw -
        cos_roll * sin_pitch * sin_yaw;
    Quaternion_Y =
        cos_roll * sin_pitch * cos_yaw +
        sin_roll * cos_pitch * sin_yaw;
    Quaternion_Z =
        cos_roll * cos_pitch * sin_yaw -
        sin_roll * sin_pitch * cos_yaw;

    Normalize_Quaternion();
    Update_Euler();
}

/**
 * @brief 归一化四元数
 */
bool Class_Attitude_Mahony::Normalize_Quaternion()
{
    const float norm_squared =
        Quaternion_W * Quaternion_W +
        Quaternion_X * Quaternion_X +
        Quaternion_Y * Quaternion_Y +
        Quaternion_Z * Quaternion_Z;

    if (norm_squared <= ATTITUDE_MAHONY_MIN_NORM_SQUARED)
    {
        Quaternion_W = 1.0f;
        Quaternion_X = 0.0f;
        Quaternion_Y = 0.0f;
        Quaternion_Z = 0.0f;
        return false;
    }

    const float inverse_norm = 1.0f / sqrtf(norm_squared);
    Quaternion_W *= inverse_norm;
    Quaternion_X *= inverse_norm;
    Quaternion_Y *= inverse_norm;
    Quaternion_Z *= inverse_norm;
    return true;
}

/**
 * @brief 由四元数更新ZYX欧拉角
 */
void Class_Attitude_Mahony::Update_Euler()
{
    const float sin_pitch =
        2.0f *
        (Quaternion_W * Quaternion_Y -
         Quaternion_Z * Quaternion_X);

    Roll = atan2f(
        2.0f *
        (Quaternion_W * Quaternion_X +
         Quaternion_Y * Quaternion_Z),
        1.0f -
        2.0f *
        (Quaternion_X * Quaternion_X +
         Quaternion_Y * Quaternion_Y));

    Pitch = asinf(Limit(sin_pitch, -1.0f, 1.0f));

    Yaw = atan2f(
        2.0f *
        (Quaternion_W * Quaternion_Z +
         Quaternion_X * Quaternion_Y),
        1.0f -
        2.0f *
        (Quaternion_Y * Quaternion_Y +
         Quaternion_Z * Quaternion_Z));
}

/**
 * @brief 用当前加速度方向初始化Roll和Pitch, Yaw置0
 */
bool Class_Attitude_Mahony::Initialize_From_Accelerometer(
    float __Accel_X,
    float __Accel_Y,
    float __Accel_Z)
{
    const float norm_squared =
        __Accel_X * __Accel_X +
        __Accel_Y * __Accel_Y +
        __Accel_Z * __Accel_Z;

    if (norm_squared <= ATTITUDE_MAHONY_MIN_NORM_SQUARED)
    {
        return false;
    }

    const float accel_norm = sqrtf(norm_squared);
    const float accel_error_ratio =
        fabsf(accel_norm - ATTITUDE_MAHONY_STANDARD_GRAVITY) /
        ATTITUDE_MAHONY_STANDARD_GRAVITY;

    if (accel_error_ratio > Accel_Rejection_Ratio)
    {
        return false;
    }

    const float roll = atan2f(__Accel_Y, __Accel_Z);
    const float pitch = atan2f(
        -__Accel_X,
        sqrtf(__Accel_Y * __Accel_Y +
              __Accel_Z * __Accel_Z));

    Set_Quaternion_From_Euler(roll, pitch, 0.0f);

    Integral_Feedback_X = 0.0f;
    Integral_Feedback_Y = 0.0f;
    Integral_Feedback_Z = 0.0f;
    Initialized_Flag = true;
    Accel_Correction_Valid_Flag = true;
    Accel_Correction_Weight = 1.0f;
    return true;
}

/**
 * @brief 执行一次Mahony六轴融合
 */
bool Class_Attitude_Mahony::Update(float __Gyro_X,
                                    float __Gyro_Y,
                                    float __Gyro_Z,
                                    float __Accel_X,
                                    float __Accel_Y,
                                    float __Accel_Z)
{
    return Update_With_D_T(__Gyro_X,
                           __Gyro_Y,
                           __Gyro_Z,
                           __Accel_X,
                           __Accel_Y,
                           __Accel_Z,
                           Sample_Period_s,
                           true);
}

/**
 * @brief 使用实际采样间隔执行一次Mahony六轴融合
 */
bool Class_Attitude_Mahony::Update_With_D_T(
    float __Gyro_X,
    float __Gyro_Y,
    float __Gyro_Z,
    float __Accel_X,
    float __Accel_Y,
    float __Accel_Z,
    float __D_T_s,
    bool __Accel_Update_Flag)
{
    if ((!std::isfinite(__Gyro_X)) ||
        (!std::isfinite(__Gyro_Y)) ||
        (!std::isfinite(__Gyro_Z)) ||
        (!std::isfinite(__D_T_s)) ||
        (__D_T_s <= 0.0f))
    {
        return false;
    }

    const bool accel_required =
        __Accel_Update_Flag || (!Initialized_Flag);

    if (accel_required &&
        ((!std::isfinite(__Accel_X)) ||
         (!std::isfinite(__Accel_Y)) ||
         (!std::isfinite(__Accel_Z))))
    {
        return false;
    }

    Sample_Period_s = __D_T_s;

    if ((!Initialized_Flag) &&
        ((!__Accel_Update_Flag) ||
         (!Initialize_From_Accelerometer(__Accel_X,
                                         __Accel_Y,
                                         __Accel_Z))))
    {
        return false;
    }

    const float accel_norm_squared =
        __Accel_X * __Accel_X +
        __Accel_Y * __Accel_Y +
        __Accel_Z * __Accel_Z;

    Accel_Correction_Valid_Flag = false;
    Accel_Correction_Weight = 0.0f;

    if (__Accel_Update_Flag &&
        (accel_norm_squared > ATTITUDE_MAHONY_MIN_NORM_SQUARED))
    {
        const float accel_norm = sqrtf(accel_norm_squared);
        const float accel_error_ratio =
            fabsf(accel_norm - ATTITUDE_MAHONY_STANDARD_GRAVITY) /
            ATTITUDE_MAHONY_STANDARD_GRAVITY;

        if (accel_error_ratio <= Accel_Rejection_Ratio)
        {
            const float inverse_accel_norm = 1.0f / accel_norm;
            const float accel_x = __Accel_X * inverse_accel_norm;
            const float accel_y = __Accel_Y * inverse_accel_norm;
            const float accel_z = __Accel_Z * inverse_accel_norm;

            // 当前四元数预测出的机体系重力单位向量
            const float gravity_x =
                2.0f *
                (Quaternion_X * Quaternion_Z -
                 Quaternion_W * Quaternion_Y);
            const float gravity_y =
                2.0f *
                (Quaternion_W * Quaternion_X +
                 Quaternion_Y * Quaternion_Z);
            const float gravity_z =
                Quaternion_W * Quaternion_W -
                Quaternion_X * Quaternion_X -
                Quaternion_Y * Quaternion_Y +
                Quaternion_Z * Quaternion_Z;

            // 测量重力方向叉乘预测重力方向
            const float error_x =
                accel_y * gravity_z -
                accel_z * gravity_y;
            const float error_y =
                accel_z * gravity_x -
                accel_x * gravity_z;
            const float error_z =
                accel_x * gravity_y -
                accel_y * gravity_x;

            if (Accel_Rejection_Ratio > 0.0f)
            {
                Accel_Correction_Weight =
                    1.0f -
                    accel_error_ratio / Accel_Rejection_Ratio;
            }
            else
            {
                Accel_Correction_Weight = 1.0f;
            }

            const float weighted_error_x =
                error_x * Accel_Correction_Weight;
            const float weighted_error_y =
                error_y * Accel_Correction_Weight;
            const float weighted_error_z =
                error_z * Accel_Correction_Weight;

            if (K_I > 0.0f)
            {
                Integral_Feedback_X +=
                    K_I * weighted_error_x * Sample_Period_s;
                Integral_Feedback_Y +=
                    K_I * weighted_error_y * Sample_Period_s;
                Integral_Feedback_Z +=
                    K_I * weighted_error_z * Sample_Period_s;

                Integral_Feedback_X =
                    Limit(Integral_Feedback_X,
                          -ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS,
                          ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS);
                Integral_Feedback_Y =
                    Limit(Integral_Feedback_Y,
                          -ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS,
                          ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS);
                Integral_Feedback_Z =
                    Limit(Integral_Feedback_Z,
                          -ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS,
                          ATTITUDE_MAHONY_INTEGRAL_LIMIT_RADPS);
            }

            __Gyro_X +=
                K_P * weighted_error_x + Integral_Feedback_X;
            __Gyro_Y +=
                K_P * weighted_error_y + Integral_Feedback_Y;
            __Gyro_Z +=
                K_P * weighted_error_z + Integral_Feedback_Z;

            Accel_Correction_Valid_Flag = true;
        }
    }

    if (__Accel_Update_Flag &&
        (!Accel_Correction_Valid_Flag))
    {
        Accel_Rejection_Count++;
    }

    const float quaternion_w = Quaternion_W;
    const float quaternion_x = Quaternion_X;
    const float quaternion_y = Quaternion_Y;
    const float quaternion_z = Quaternion_Z;
    const float half_period = 0.5f * Sample_Period_s;

    Quaternion_W +=
        (-quaternion_x * __Gyro_X -
         quaternion_y * __Gyro_Y -
         quaternion_z * __Gyro_Z) *
        half_period;
    Quaternion_X +=
        (quaternion_w * __Gyro_X +
         quaternion_y * __Gyro_Z -
         quaternion_z * __Gyro_Y) *
        half_period;
    Quaternion_Y +=
        (quaternion_w * __Gyro_Y -
         quaternion_x * __Gyro_Z +
         quaternion_z * __Gyro_X) *
        half_period;
    Quaternion_Z +=
        (quaternion_w * __Gyro_Z +
         quaternion_x * __Gyro_Y -
         quaternion_y * __Gyro_X) *
        half_period;

    if (!Normalize_Quaternion())
    {
        Initialized_Flag = false;
        return false;
    }

    Update_Euler();
    Update_Count++;
    return true;
}

/**
 * @brief 保持当前Roll和Pitch并将Yaw重新置0
 */
void Class_Attitude_Mahony::Reset_Yaw()
{
    if (!Initialized_Flag)
    {
        return;
    }

    Set_Quaternion_From_Euler(Roll, Pitch, 0.0f);
    Integral_Feedback_Z = 0.0f;
}

/*----------------------------------------------------------------------------*/
