/**
 * @file alg_attitude_ekf.h
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 基于达妙MC02 BSP模型的四元数EKF姿态解算
 * @version 1.0
 * @date 2026-07-24 27赛季
 *
 * @note 状态为四元数4维, 输入为三轴角速度, 观测为归一化加速度.
 *       使用卡方检验拒绝不可信的加速度修正.
 */

#ifndef ALG_ATTITUDE_EKF_H
#define ALG_ATTITUDE_EKF_H

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_ekf.h"
#include "alg_quaternion.h"

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define ATTITUDE_EKF_PI                    (3.14159265358979323846f)
#define ATTITUDE_EKF_RAD_TO_DEGREE         \
    (180.0f / ATTITUDE_EKF_PI)
#define ATTITUDE_EKF_MIN_NORM_SQUARED      (1.0e-12f)
#define ATTITUDE_EKF_DEFAULT_PERIOD_S      (0.001f)
#define ATTITUDE_EKF_DEFAULT_CHI_SQUARE_THRESHOLD (0.5f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四元数扩展Kalman姿态解算器
 *
 * @note 坐标定义与当前工程保持一致:
 *       Roll绕开发板+X, Pitch绕开发板+Y, Yaw绕开发板+Z.
 */
class Class_Attitude_QuaternionEKF
{
public:
    void Init(
        float __Sample_Period_s = ATTITUDE_EKF_DEFAULT_PERIOD_S,
        float __Chi_Square_Threshold =
            ATTITUDE_EKF_DEFAULT_CHI_SQUARE_THRESHOLD);

    void Reset();

    bool Update(float __Gyro_X,
                float __Gyro_Y,
                float __Gyro_Z,
                float __Accel_X,
                float __Accel_Y,
                float __Accel_Z);

    /**
     * @brief 使用实际采样间隔执行一次EKF
     *
     * @param __D_T_s 本次陀螺仪样本间隔, 单位s
     * @param __Accel_Update_Flag 本次是否有新的加速度计观测
     */
    bool Update_With_D_T(float __Gyro_X,
                         float __Gyro_Y,
                         float __Gyro_Z,
                         float __Accel_X,
                         float __Accel_Y,
                         float __Accel_Z,
                         float __D_T_s,
                         bool __Accel_Update_Flag);

    void Reset_Yaw();

    inline bool Get_Initialized_Flag() const;
    inline bool Get_Accel_Correction_Valid_Flag() const;
    inline uint32_t Get_Update_Count() const;
    inline uint32_t Get_Accel_Rejection_Count() const;
    inline float Get_Accel_Chi_Square() const;
    inline float Get_Quaternion_W() const;
    inline float Get_Quaternion_X() const;
    inline float Get_Quaternion_Y() const;
    inline float Get_Quaternion_Z() const;
    inline Class_Quaternion_f32 Get_Quaternion() const;
    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;
    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;
    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;
    inline float Get_Roll() const;
    inline float Get_Pitch() const;
    inline float Get_Yaw() const;
    inline float Get_Roll_Degree() const;
    inline float Get_Pitch_Degree() const;
    inline float Get_Yaw_Degree() const;

protected:
    Class_Filter_EKF<4, 3, 3> EKF_Quaternion;

    float Sample_Period_s = ATTITUDE_EKF_DEFAULT_PERIOD_S;
    float Chi_Square_Threshold =
        ATTITUDE_EKF_DEFAULT_CHI_SQUARE_THRESHOLD;

    Class_Quaternion_f32 Quaternion;
    Class_Matrix_f32<3, 1> Euler_Angle;

    bool Initialized_Flag = false;
    bool Accel_Correction_Valid_Flag = false;
    uint32_t Update_Count = 0U;
    uint32_t Accel_Rejection_Count = 0U;
    float Accel_Chi_Square = 0.0f;

    bool Initialize_From_Accelerometer(float __Accel_X,
                                       float __Accel_Y,
                                       float __Accel_Z);
    bool Calculate_Accel_Chi_Square(
        const Class_Matrix_f32<3, 1> &__Normalized_Accel,
        float *__Chi_Square) const;

    static Class_Matrix_f32<4, 1> EKF_Function_F(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const Class_Matrix_f32<3, 1> &__Vector_U,
        const float &__D_T);

    static Class_Matrix_f32<4, 4>
    EKF_Function_Jacobian_F_X(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const Class_Matrix_f32<3, 1> &__Vector_U,
        const float &__D_T);

    static Class_Matrix_f32<4, 3>
    EKF_Function_Jacobian_F_W(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const Class_Matrix_f32<3, 1> &__Vector_U,
        const float &__D_T);

    static Class_Matrix_f32<3, 1> EKF_Function_H(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const float &__D_T);

    static Class_Matrix_f32<3, 4>
    EKF_Function_Jacobian_H_X(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const float &__D_T);

    static Class_Matrix_f32<3, 3>
    EKF_Function_Jacobian_H_V(
        const Class_Matrix_f32<4, 1> &__Vector_X,
        const float &__D_T);

    static Class_Matrix_f32<4, 4> Get_Omega_Matrix(
        const Class_Matrix_f32<3, 1> &__Vector_U);

    static Class_Matrix_f32<4, 4> Get_Normalization_Jacobian(
        const Class_Matrix_f32<4, 1> &__Quaternion_Tmp);
};

/* Exported function definitions ---------------------------------------------*/

inline bool
Class_Attitude_QuaternionEKF::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline bool
Class_Attitude_QuaternionEKF::
Get_Accel_Correction_Valid_Flag() const
{
    return Accel_Correction_Valid_Flag;
}

inline uint32_t
Class_Attitude_QuaternionEKF::Get_Update_Count() const
{
    return Update_Count;
}

inline uint32_t
Class_Attitude_QuaternionEKF::Get_Accel_Rejection_Count() const
{
    return Accel_Rejection_Count;
}

inline float
Class_Attitude_QuaternionEKF::Get_Accel_Chi_Square() const
{
    return Accel_Chi_Square;
}

inline float
Class_Attitude_QuaternionEKF::Get_Quaternion_W() const
{
    return Quaternion[0];
}

inline float
Class_Attitude_QuaternionEKF::Get_Quaternion_X() const
{
    return Quaternion[1];
}

inline float
Class_Attitude_QuaternionEKF::Get_Quaternion_Y() const
{
    return Quaternion[2];
}

inline float
Class_Attitude_QuaternionEKF::Get_Quaternion_Z() const
{
    return Quaternion[3];
}

inline Class_Quaternion_f32
Class_Attitude_QuaternionEKF::Get_Quaternion() const
{
    return Quaternion;
}

/**
 * @brief 获取ZYX欧拉角, 顺序为Yaw、Pitch、Roll
 */
inline Class_Matrix_f32<3, 1>
Class_Attitude_QuaternionEKF::Get_Euler_Angle() const
{
    return Euler_Angle;
}

/**
 * @brief 获取机体系到世界系的旋转矩阵
 */
inline Class_Matrix_f32<3, 3>
Class_Attitude_QuaternionEKF::Get_Rotation_Matrix() const
{
    return Quaternion.Get_Rotation_Matrix();
}

inline Class_Matrix_f32<4, 1>
Class_Attitude_QuaternionEKF::Get_Axis_Angle() const
{
    return Quaternion.Get_Axis_Angle();
}

inline float Class_Attitude_QuaternionEKF::Get_Roll() const
{
    return Euler_Angle[2][0];
}

inline float Class_Attitude_QuaternionEKF::Get_Pitch() const
{
    return Euler_Angle[1][0];
}

inline float Class_Attitude_QuaternionEKF::Get_Yaw() const
{
    return Euler_Angle[0][0];
}

inline float
Class_Attitude_QuaternionEKF::Get_Roll_Degree() const
{
    return Get_Roll() * ATTITUDE_EKF_RAD_TO_DEGREE;
}

inline float
Class_Attitude_QuaternionEKF::Get_Pitch_Degree() const
{
    return Get_Pitch() * ATTITUDE_EKF_RAD_TO_DEGREE;
}

inline float
Class_Attitude_QuaternionEKF::Get_Yaw_Degree() const
{
    return Get_Yaw() * ATTITUDE_EKF_RAD_TO_DEGREE;
}

#endif /* ALG_ATTITUDE_EKF_H */

/*----------------------------------------------------------------------------*/
