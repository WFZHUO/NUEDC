/**
 * @file alg_filter_ekf.h
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 固定尺寸扩展Kalman滤波器
 * @version 1.0
 * @date 2026-07-24 由达妙MC02 BSP EKF库适配
 *
 * @copyright USTC-RoboWalker (c) 2025
 */

#ifndef ALG_FILTER_EKF_H
#define ALG_FILTER_EKF_H

/* Includes ------------------------------------------------------------------*/

#include "alg_matrix.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 固定尺寸扩展Kalman滤波器
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 *
 * @note 与达妙源库一致采用Joseph形式更新协方差.
 *       全部矩阵静态分配, 运行时不申请堆内存.
 */
template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
class Class_Filter_EKF
{
public:
    using State_Vector =
        Class_Matrix_f32<State_Dimension, 1>;
    using Input_Vector =
        Class_Matrix_f32<Input_Dimension, 1>;
    using Measurement_Vector =
        Class_Matrix_f32<Measurement_Dimension, 1>;
    using State_Matrix =
        Class_Matrix_f32<State_Dimension, State_Dimension>;
    using Process_Noise_Matrix =
        Class_Matrix_f32<Input_Dimension, Input_Dimension>;
    using Measurement_Noise_Matrix =
        Class_Matrix_f32<Measurement_Dimension,
                         Measurement_Dimension>;

    State_Vector (*Function_F)(
        const State_Vector &__Vector_X,
        const Input_Vector &__Vector_U,
        const float &__D_T) = nullptr;

    State_Matrix (*Function_Jacobian_F_X)(
        const State_Vector &__Vector_X,
        const Input_Vector &__Vector_U,
        const float &__D_T) = nullptr;

    Class_Matrix_f32<State_Dimension, Input_Dimension>
    (*Function_Jacobian_F_W)(
        const State_Vector &__Vector_X,
        const Input_Vector &__Vector_U,
        const float &__D_T) = nullptr;

    Measurement_Vector (*Function_H)(
        const State_Vector &__Vector_X,
        const float &__D_T) = nullptr;

    Class_Matrix_f32<Measurement_Dimension, State_Dimension>
    (*Function_Jacobian_H_X)(
        const State_Vector &__Vector_X,
        const float &__D_T) = nullptr;

    Measurement_Noise_Matrix (*Function_Jacobian_H_V)(
        const State_Vector &__Vector_X,
        const float &__D_T) = nullptr;

    Process_Noise_Matrix Matrix_Q;
    Measurement_Noise_Matrix Matrix_R;
    State_Matrix Matrix_P;
    State_Vector Vector_X;
    Input_Vector Vector_U;
    State_Vector Vector_X_Prior;
    State_Matrix Matrix_P_Prior;
    Class_Matrix_f32<State_Dimension, Measurement_Dimension>
        Matrix_K;
    Measurement_Vector Vector_Z;

    void Init(const Process_Noise_Matrix &__Matrix_Q,
              const Measurement_Noise_Matrix &__Matrix_R,
              const State_Matrix &__Matrix_P =
                  Namespace_ALG_Matrix::Identity<
                      State_Dimension,
                      State_Dimension>(),
              const State_Vector &__Vector_X =
                  Namespace_ALG_Matrix::Zero<
                      State_Dimension,
                      1>(),
              const Input_Vector &__Vector_U =
                  Namespace_ALG_Matrix::Zero<
                      Input_Dimension,
                      1>());

    void Config_Nonlinear_State_Model(
        State_Vector (*__Function_F)(
            const State_Vector &,
            const Input_Vector &,
            const float &),
        State_Matrix (*__Function_Jacobian_F_X)(
            const State_Vector &,
            const Input_Vector &,
            const float &),
        Class_Matrix_f32<State_Dimension, Input_Dimension>
        (*__Function_Jacobian_F_W)(
            const State_Vector &,
            const Input_Vector &,
            const float &));

    void Config_Nonlinear_Measurement_Model(
        Measurement_Vector (*__Function_H)(
            const State_Vector &,
            const float &),
        Class_Matrix_f32<Measurement_Dimension, State_Dimension>
        (*__Function_Jacobian_H_X)(
            const State_Vector &,
            const float &),
        Measurement_Noise_Matrix
        (*__Function_Jacobian_H_V)(
            const State_Vector &,
            const float &));

    inline void Set_D_T(float __D_T)
    {
        D_T = __D_T;
    }

    bool TIM_Predict_PeriodElapsedCallback();
    bool TIM_Update_PeriodElapsedCallback();

protected:
    const State_Matrix MATRIX_I_STATE =
        Namespace_ALG_Matrix::Identity<
            State_Dimension,
            State_Dimension>();
    float D_T = 0.001f;
};

/* Exported function definitions ---------------------------------------------*/

template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
void Class_Filter_EKF<
    State_Dimension,
    Input_Dimension,
    Measurement_Dimension>::Init(
    const Process_Noise_Matrix &__Matrix_Q,
    const Measurement_Noise_Matrix &__Matrix_R,
    const State_Matrix &__Matrix_P,
    const State_Vector &__Vector_X,
    const Input_Vector &__Vector_U)
{
    Matrix_Q = __Matrix_Q;
    Matrix_R = __Matrix_R;
    Matrix_P = __Matrix_P;
    Vector_X = __Vector_X;
    Vector_X_Prior = __Vector_X;
    Matrix_P_Prior = __Matrix_P;
    Vector_U = __Vector_U;
}

template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
void Class_Filter_EKF<
    State_Dimension,
    Input_Dimension,
    Measurement_Dimension>::Config_Nonlinear_State_Model(
    State_Vector (*__Function_F)(
        const State_Vector &,
        const Input_Vector &,
        const float &),
    State_Matrix (*__Function_Jacobian_F_X)(
        const State_Vector &,
        const Input_Vector &,
        const float &),
    Class_Matrix_f32<State_Dimension, Input_Dimension>
    (*__Function_Jacobian_F_W)(
        const State_Vector &,
        const Input_Vector &,
        const float &))
{
    Function_F = __Function_F;
    Function_Jacobian_F_X = __Function_Jacobian_F_X;
    Function_Jacobian_F_W = __Function_Jacobian_F_W;
}

template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
void Class_Filter_EKF<
    State_Dimension,
    Input_Dimension,
    Measurement_Dimension>::
Config_Nonlinear_Measurement_Model(
    Measurement_Vector (*__Function_H)(
        const State_Vector &,
        const float &),
    Class_Matrix_f32<Measurement_Dimension, State_Dimension>
    (*__Function_Jacobian_H_X)(
        const State_Vector &,
        const float &),
    Measurement_Noise_Matrix
    (*__Function_Jacobian_H_V)(
        const State_Vector &,
        const float &))
{
    Function_H = __Function_H;
    Function_Jacobian_H_X = __Function_Jacobian_H_X;
    Function_Jacobian_H_V = __Function_Jacobian_H_V;
}

template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
bool Class_Filter_EKF<
    State_Dimension,
    Input_Dimension,
    Measurement_Dimension>::
TIM_Predict_PeriodElapsedCallback()
{
    if ((Function_F == nullptr) ||
        (Function_Jacobian_F_X == nullptr) ||
        (Function_Jacobian_F_W == nullptr) ||
        (!std::isfinite(D_T)) ||
        (D_T <= 0.0f))
    {
        return false;
    }

    // 先执行状态函数. 达妙BMI088的四元数模型会在此计算
    // 归一化Jacobi矩阵, 随后的两个Jacobi函数需要使用它.
    Vector_X_Prior = Function_F(Vector_X, Vector_U, D_T);

    const State_Matrix matrix_f_x =
        Function_Jacobian_F_X(Vector_X, Vector_U, D_T);
    const Class_Matrix_f32<State_Dimension, Input_Dimension>
        matrix_f_w =
            Function_Jacobian_F_W(Vector_X, Vector_U, D_T);

    Matrix_P_Prior =
        matrix_f_x * Matrix_P * matrix_f_x.Get_Transpose() +
        matrix_f_w * Matrix_Q * matrix_f_w.Get_Transpose();

    // 没有新观测时也保留预测结果.
    Vector_X = Vector_X_Prior;
    Matrix_P = Matrix_P_Prior;
    return true;
}

template<int State_Dimension,
         int Input_Dimension,
         int Measurement_Dimension>
bool Class_Filter_EKF<
    State_Dimension,
    Input_Dimension,
    Measurement_Dimension>::
TIM_Update_PeriodElapsedCallback()
{
    if ((Function_H == nullptr) ||
        (Function_Jacobian_H_X == nullptr) ||
        (Function_Jacobian_H_V == nullptr))
    {
        return false;
    }

    const Class_Matrix_f32<
        Measurement_Dimension,
        State_Dimension> matrix_h_x =
            Function_Jacobian_H_X(Vector_X_Prior, D_T);
    const Measurement_Noise_Matrix matrix_h_v =
        Function_Jacobian_H_V(Vector_X_Prior, D_T);

    const Measurement_Noise_Matrix matrix_s =
        matrix_h_x *
            Matrix_P_Prior *
            matrix_h_x.Get_Transpose() +
        matrix_h_v *
            Matrix_R *
            matrix_h_v.Get_Transpose();

    bool inverse_success = false;
    const Measurement_Noise_Matrix matrix_s_inverse =
        matrix_s.Get_Inverse(&inverse_success);

    if (!inverse_success)
    {
        return false;
    }

    Matrix_K =
        Matrix_P_Prior *
        matrix_h_x.Get_Transpose() *
        matrix_s_inverse;

    Vector_X =
        Vector_X_Prior +
        Matrix_K *
            (Vector_Z - Function_H(Vector_X_Prior, D_T));

    // Joseph形式在单精度下比(I-KH)P更稳定.
    const State_Matrix matrix_tmp =
        MATRIX_I_STATE - Matrix_K * matrix_h_x;

    Matrix_P =
        matrix_tmp *
            Matrix_P_Prior *
            matrix_tmp.Get_Transpose() +
        Matrix_K *
            matrix_h_v *
            Matrix_R *
            matrix_h_v.Get_Transpose() *
            Matrix_K.Get_Transpose();

    return true;
}

#endif /* ALG_FILTER_EKF_H */

/*----------------------------------------------------------------------------*/
