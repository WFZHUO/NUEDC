/**
 * @file alg_attitude_ekf.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 基于达妙MC02 BSP模型的四元数EKF姿态解算
 * @version 1.0
 * @date 2026-07-24 27赛季
 */

#include "alg_attitude_ekf.h"

#include <cmath>
#include <limits>

/* Private variables ---------------------------------------------------------*/

// 达妙源参数按照当前开发板坐标映射:
// Board X = IMU Y, Board Y = -IMU X, Board Z = IMU Z.
static const float ATTITUDE_EKF_Q_DATA[9] =
{
    0.975f, 0.0f, 0.0f,
    0.0f, 0.865f, 0.0f,
    0.0f, 0.0f, 1.077f,
};

static const float ATTITUDE_EKF_R_DATA[9] =
{
    0.476f, 0.0f, 0.0f,
    0.0f, 0.446f, 0.0f,
    0.0f, 0.0f, 0.537f,
};

/* Function definitions ------------------------------------------------------*/

void Class_Attitude_QuaternionEKF::Init(
    float __Sample_Period_s,
    float __Chi_Square_Threshold)
{
    if ((!std::isfinite(__Sample_Period_s)) ||
        (__Sample_Period_s <= 0.0f))
    {
        __Sample_Period_s = ATTITUDE_EKF_DEFAULT_PERIOD_S;
    }

    if ((!std::isfinite(__Chi_Square_Threshold)) ||
        (__Chi_Square_Threshold <= 0.0f))
    {
        __Chi_Square_Threshold =
            ATTITUDE_EKF_DEFAULT_CHI_SQUARE_THRESHOLD;
    }

    Sample_Period_s = __Sample_Period_s;
    Chi_Square_Threshold = __Chi_Square_Threshold;
    Reset();
}

void Class_Attitude_QuaternionEKF::Reset()
{
    Quaternion = Class_Quaternion_f32();
    Euler_Angle = Class_Matrix_f32<3, 1>();
    Initialized_Flag = false;
    Accel_Correction_Valid_Flag = false;
    Update_Count = 0U;
    Accel_Rejection_Count = 0U;
    Accel_Chi_Square = 0.0f;
}

bool Class_Attitude_QuaternionEKF::Initialize_From_Accelerometer(
    float __Accel_X,
    float __Accel_Y,
    float __Accel_Z)
{
    Class_Matrix_f32<3, 1> accel;
    accel[0][0] = __Accel_X;
    accel[1][0] = __Accel_Y;
    accel[2][0] = __Accel_Z;

    bool normalize_success = false;
    const Class_Matrix_f32<3, 1> normalized_accel =
        accel.Get_Normalization(&normalize_success);

    if (!normalize_success)
    {
        return false;
    }

    Class_Matrix_f32<3, 1> initial_euler;
    initial_euler[0][0] = 0.0f;

    float pitch_argument = -normalized_accel[0][0];
    pitch_argument =
        std::max(-1.0f, std::min(1.0f, pitch_argument));
    initial_euler[1][0] = std::asin(pitch_argument);
    initial_euler[2][0] =
        std::atan2(normalized_accel[1][0],
                   normalized_accel[2][0]);

    Quaternion =
        Namespace_ALG_Quaternion::From_Euler_Angle(
            initial_euler)
            .Get_Normalization(&normalize_success);

    if (!normalize_success)
    {
        return false;
    }

    const Class_Matrix_f32<3, 3> matrix_q(
        ATTITUDE_EKF_Q_DATA);
    const Class_Matrix_f32<3, 3> matrix_r(
        ATTITUDE_EKF_R_DATA);
    const Class_Matrix_f32<4, 4> matrix_p =
        Namespace_ALG_Matrix::Identity<4, 4>();

    EKF_Quaternion.Init(
        matrix_q,
        matrix_r,
        matrix_p,
        static_cast<const Class_Matrix_f32<4, 1> &>(
            Quaternion));

    EKF_Quaternion.Config_Nonlinear_State_Model(
        EKF_Function_F,
        EKF_Function_Jacobian_F_X,
        EKF_Function_Jacobian_F_W);
    EKF_Quaternion.Config_Nonlinear_Measurement_Model(
        EKF_Function_H,
        EKF_Function_Jacobian_H_X,
        EKF_Function_Jacobian_H_V);
    EKF_Quaternion.Set_D_T(Sample_Period_s);

    Euler_Angle = Quaternion.Get_Euler_Angle();
    Initialized_Flag = true;
    Accel_Correction_Valid_Flag = true;
    return true;
}

bool Class_Attitude_QuaternionEKF::Update(
    float __Gyro_X,
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

bool Class_Attitude_QuaternionEKF::Update_With_D_T(
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
        (!std::isfinite(__Gyro_Z)))
    {
        return false;
    }

    if ((!std::isfinite(__D_T_s)) || (__D_T_s <= 0.0f))
    {
        return false;
    }

    Sample_Period_s = __D_T_s;

    const bool accel_required =
        __Accel_Update_Flag || (!Initialized_Flag);

    if (accel_required &&
        ((!std::isfinite(__Accel_X)) ||
         (!std::isfinite(__Accel_Y)) ||
         (!std::isfinite(__Accel_Z))))
    {
        return false;
    }

    if ((!Initialized_Flag) &&
        ((!__Accel_Update_Flag) ||
         (!Initialize_From_Accelerometer(
             __Accel_X,
             __Accel_Y,
             __Accel_Z))))
    {
        return false;
    }

    EKF_Quaternion.Vector_U[0][0] = __Gyro_X;
    EKF_Quaternion.Vector_U[1][0] = __Gyro_Y;
    EKF_Quaternion.Vector_U[2][0] = __Gyro_Z;
    EKF_Quaternion.Set_D_T(Sample_Period_s);

    if (!EKF_Quaternion.TIM_Predict_PeriodElapsedCallback())
    {
        Reset();
        return false;
    }

    Class_Matrix_f32<3, 1> accel;
    accel[0][0] = __Accel_X;
    accel[1][0] = __Accel_Y;
    accel[2][0] = __Accel_Z;

    bool normalize_success = false;
    const Class_Matrix_f32<3, 1> normalized_accel =
        accel.Get_Normalization(&normalize_success);

    Accel_Correction_Valid_Flag = false;

    if (__Accel_Update_Flag &&
        normalize_success &&
        Calculate_Accel_Chi_Square(
            normalized_accel,
            &Accel_Chi_Square) &&
        (Accel_Chi_Square <= Chi_Square_Threshold))
    {
        EKF_Quaternion.Vector_Z = normalized_accel;
        Accel_Correction_Valid_Flag =
            EKF_Quaternion.TIM_Update_PeriodElapsedCallback();
    }

    if (__Accel_Update_Flag &&
        (!Accel_Correction_Valid_Flag))
    {
        Accel_Rejection_Count++;
    }

    const Class_Matrix_f32<4, 1> quaternion_before_normalization =
        EKF_Quaternion.Vector_X;
    const Class_Matrix_f32<4, 4> normalization_jacobian =
        Get_Normalization_Jacobian(
            quaternion_before_normalization);

    bool quaternion_valid = false;
    Quaternion =
        Class_Quaternion_f32(quaternion_before_normalization)
            .Get_Normalization(&quaternion_valid);

    if (!quaternion_valid)
    {
        Reset();
        return false;
    }

    // 将归一化后的状态及其协方差反馈给下一轮预测.
    EKF_Quaternion.Vector_X =
        static_cast<const Class_Matrix_f32<4, 1> &>(
            Quaternion);
    EKF_Quaternion.Matrix_P =
        normalization_jacobian *
        EKF_Quaternion.Matrix_P *
        normalization_jacobian.Get_Transpose();

    Euler_Angle = Quaternion.Get_Euler_Angle();
    Update_Count++;
    return true;
}

void Class_Attitude_QuaternionEKF::Reset_Yaw()
{
    if (!Initialized_Flag)
    {
        return;
    }

    Class_Matrix_f32<3, 1> euler = Euler_Angle;
    euler[0][0] = 0.0f;

    bool normalize_success = false;
    Quaternion =
        Namespace_ALG_Quaternion::From_Euler_Angle(euler)
            .Get_Normalization(&normalize_success);

    if (!normalize_success)
    {
        Reset();
        return;
    }

    EKF_Quaternion.Vector_X =
        static_cast<const Class_Matrix_f32<4, 1> &>(
            Quaternion);
    EKF_Quaternion.Vector_X_Prior = EKF_Quaternion.Vector_X;
    EKF_Quaternion.Matrix_P =
        Namespace_ALG_Matrix::Identity<4, 4>();
    EKF_Quaternion.Matrix_P_Prior = EKF_Quaternion.Matrix_P;
    Euler_Angle = Quaternion.Get_Euler_Angle();
}

bool Class_Attitude_QuaternionEKF::Calculate_Accel_Chi_Square(
    const Class_Matrix_f32<3, 1> &__Normalized_Accel,
    float *__Chi_Square) const
{
    if (__Chi_Square == nullptr)
    {
        return false;
    }

    const Class_Matrix_f32<3, 1> residual =
        __Normalized_Accel -
        EKF_Function_H(
            EKF_Quaternion.Vector_X_Prior,
            Sample_Period_s);
    const Class_Matrix_f32<3, 4> matrix_h_x =
        EKF_Function_Jacobian_H_X(
            EKF_Quaternion.Vector_X_Prior,
            Sample_Period_s);
    const Class_Matrix_f32<3, 3> matrix_s =
        matrix_h_x *
            EKF_Quaternion.Matrix_P_Prior *
            matrix_h_x.Get_Transpose() +
        EKF_Quaternion.Matrix_R;

    bool inverse_success = false;
    const Class_Matrix_f32<3, 3> matrix_s_inverse =
        matrix_s.Get_Inverse(&inverse_success);

    if (!inverse_success)
    {
        *__Chi_Square =
            std::numeric_limits<float>::infinity();
        return false;
    }

    const Class_Matrix_f32<1, 1> chi_square =
        residual.Get_Transpose() *
        matrix_s_inverse *
        residual;
    *__Chi_Square = chi_square[0][0];

    return std::isfinite(*__Chi_Square);
}

Class_Matrix_f32<4, 4>
Class_Attitude_QuaternionEKF::Get_Omega_Matrix(
    const Class_Matrix_f32<3, 1> &__Vector_U)
{
    const float gx = __Vector_U[0][0];
    const float gy = __Vector_U[1][0];
    const float gz = __Vector_U[2][0];
    Class_Matrix_f32<4, 4> omega;

    omega[0][1] = -gx;
    omega[0][2] = -gy;
    omega[0][3] = -gz;
    omega[1][0] = gx;
    omega[1][2] = gz;
    omega[1][3] = -gy;
    omega[2][0] = gy;
    omega[2][1] = -gz;
    omega[2][3] = gx;
    omega[3][0] = gz;
    omega[3][1] = gy;
    omega[3][2] = -gx;

    return omega;
}

Class_Matrix_f32<4, 4>
Class_Attitude_QuaternionEKF::Get_Normalization_Jacobian(
    const Class_Matrix_f32<4, 1> &__Quaternion_Tmp)
{
    const float modulus =
        __Quaternion_Tmp.Get_Modulus();

    if ((!std::isfinite(modulus)) ||
        (modulus <= Matrix_Compare_Epsilon))
    {
        return Class_Matrix_f32<4, 4>();
    }

    const float inverse_modulus = 1.0f / modulus;
    return inverse_modulus *
           (Namespace_ALG_Matrix::Identity<4, 4>() -
            (inverse_modulus * inverse_modulus) *
                __Quaternion_Tmp *
                __Quaternion_Tmp.Get_Transpose());
}

Class_Matrix_f32<4, 1>
Class_Attitude_QuaternionEKF::EKF_Function_F(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const Class_Matrix_f32<3, 1> &__Vector_U,
    const float &__D_T)
{
    const Class_Matrix_f32<4, 4> omega =
        Get_Omega_Matrix(__Vector_U);
    const Class_Matrix_f32<4, 1> quaternion_tmp =
        __Vector_X + 0.5f * __D_T * omega * __Vector_X;
    return quaternion_tmp.Get_Normalization();
}

Class_Matrix_f32<4, 4>
Class_Attitude_QuaternionEKF::EKF_Function_Jacobian_F_X(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const Class_Matrix_f32<3, 1> &__Vector_U,
    const float &__D_T)
{
    const Class_Matrix_f32<4, 4> omega =
        Get_Omega_Matrix(__Vector_U);
    const Class_Matrix_f32<4, 4> state_transition =
        Namespace_ALG_Matrix::Identity<4, 4>() +
        0.5f * __D_T * omega;
    const Class_Matrix_f32<4, 1> quaternion_tmp =
        state_transition * __Vector_X;
    const Class_Matrix_f32<4, 4> normalization_jacobian =
        Get_Normalization_Jacobian(quaternion_tmp);

    return normalization_jacobian * state_transition;
}

Class_Matrix_f32<4, 3>
Class_Attitude_QuaternionEKF::EKF_Function_Jacobian_F_W(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const Class_Matrix_f32<3, 1> &__Vector_U,
    const float &__D_T)
{
    const Class_Matrix_f32<4, 4> omega =
        Get_Omega_Matrix(__Vector_U);
    const Class_Matrix_f32<4, 1> quaternion_tmp =
        __Vector_X + 0.5f * __D_T * omega * __Vector_X;
    const Class_Matrix_f32<4, 4> normalization_jacobian =
        Get_Normalization_Jacobian(quaternion_tmp);

    Class_Matrix_f32<4, 3> matrix_q;
    const float q0 = __Vector_X[0][0];
    const float q1 = __Vector_X[1][0];
    const float q2 = __Vector_X[2][0];
    const float q3 = __Vector_X[3][0];

    matrix_q[0][0] = -q1;
    matrix_q[0][1] = -q2;
    matrix_q[0][2] = -q3;
    matrix_q[1][0] = q0;
    matrix_q[1][1] = -q3;
    matrix_q[1][2] = q2;
    matrix_q[2][0] = q3;
    matrix_q[2][1] = q0;
    matrix_q[2][2] = -q1;
    matrix_q[3][0] = -q2;
    matrix_q[3][1] = q1;
    matrix_q[3][2] = q0;

    return normalization_jacobian *
           (0.5f * __D_T * matrix_q);
}

Class_Matrix_f32<3, 1>
Class_Attitude_QuaternionEKF::EKF_Function_H(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const float &__D_T)
{
    (void)__D_T;
    Class_Matrix_f32<3, 1> result;
    const float q0 = __Vector_X[0][0];
    const float q1 = __Vector_X[1][0];
    const float q2 = __Vector_X[2][0];
    const float q3 = __Vector_X[3][0];

    result[0][0] = 2.0f * (q1 * q3 - q0 * q2);
    result[1][0] = 2.0f * (q2 * q3 + q0 * q1);
    result[2][0] =
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    return result;
}

Class_Matrix_f32<3, 4>
Class_Attitude_QuaternionEKF::EKF_Function_Jacobian_H_X(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const float &__D_T)
{
    (void)__D_T;
    Class_Matrix_f32<3, 4> result;
    const float q0 = __Vector_X[0][0];
    const float q1 = __Vector_X[1][0];
    const float q2 = __Vector_X[2][0];
    const float q3 = __Vector_X[3][0];

    result[0][0] = -2.0f * q2;
    result[0][1] = 2.0f * q3;
    result[0][2] = -2.0f * q0;
    result[0][3] = 2.0f * q1;
    result[1][0] = 2.0f * q1;
    result[1][1] = 2.0f * q0;
    result[1][2] = 2.0f * q3;
    result[1][3] = 2.0f * q2;
    result[2][0] = 2.0f * q0;
    result[2][1] = -2.0f * q1;
    result[2][2] = -2.0f * q2;
    result[2][3] = 2.0f * q3;

    return result;
}

Class_Matrix_f32<3, 3>
Class_Attitude_QuaternionEKF::EKF_Function_Jacobian_H_V(
    const Class_Matrix_f32<4, 1> &__Vector_X,
    const float &__D_T)
{
    (void)__Vector_X;
    (void)__D_T;
    return Namespace_ALG_Matrix::Identity<3, 3>();
}

/*----------------------------------------------------------------------------*/
