/**
 * @file alg_quaternion.h
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 四元数计算支持库
 * @version 1.0
 * @date 2026-07-24 由达妙MC02 BSP四元数库适配
 *
 * @copyright USTC-RoboWalker (c) 2025
 */

#ifndef ALG_QUATERNION_H
#define ALG_QUATERNION_H

/* Includes ------------------------------------------------------------------*/

#include "alg_matrix.h"

#include <algorithm>
#include <cmath>

/* Exported macros -----------------------------------------------------------*/

#define ALG_QUATERNION_PI (3.14159265358979323846f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四元数, 数据顺序为W、X、Y、Z
 */
class Class_Quaternion_f32 : public Class_Matrix_f32<4, 1>
{
public:
    Class_Quaternion_f32(float __W = 1.0f,
                         float __X = 0.0f,
                         float __Y = 0.0f,
                         float __Z = 0.0f)
    {
        Data[0] = __W;
        Data[1] = __X;
        Data[2] = __Y;
        Data[3] = __Z;
    }

    explicit Class_Quaternion_f32(
        const Class_Matrix_f32<4, 1> &__Matrix)
    {
        std::memcpy(Data, __Matrix.Data, sizeof(Data));
    }

    inline float &operator[](int __Index)
    {
        return Data[__Index];
    }

    inline const float &operator[](int __Index) const
    {
        return Data[__Index];
    }

    inline Class_Quaternion_f32 Get_Normalization(
        bool *__Success = nullptr) const
    {
        const Class_Matrix_f32<4, 1> normalized =
            Class_Matrix_f32<4, 1>::Get_Normalization(__Success);
        return Class_Quaternion_f32(normalized);
    }

    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const
    {
        Class_Matrix_f32<3, 1> result;
        bool normalized_success = false;
        const Class_Quaternion_f32 quaternion =
            Get_Normalization(&normalized_success);

        if (!normalized_success)
        {
            return result;
        }

        const float q0 = quaternion[0];
        const float q1 = quaternion[1];
        const float q2 = quaternion[2];
        const float q3 = quaternion[3];

        // Z-Y-X顺序, 返回Yaw、Pitch、Roll
        result[0][0] =
            std::atan2(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3));

        float sin_pitch =
            2.0f * (q0 * q2 - q3 * q1);
        sin_pitch = std::max(-1.0f, std::min(1.0f, sin_pitch));
        result[1][0] = std::asin(sin_pitch);

        result[2][0] =
            std::atan2(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));

        return result;
    }

    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const
    {
        Class_Matrix_f32<3, 3> result =
            Namespace_ALG_Matrix::Identity<3, 3>();
        bool normalized_success = false;
        const Class_Quaternion_f32 quaternion =
            Get_Normalization(&normalized_success);

        if (!normalized_success)
        {
            return result;
        }

        const float q0 = quaternion[0];
        const float q1 = quaternion[1];
        const float q2 = quaternion[2];
        const float q3 = quaternion[3];

        result[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        result[0][1] = 2.0f * (q1 * q2 - q0 * q3);
        result[0][2] = 2.0f * (q1 * q3 + q0 * q2);
        result[1][0] = 2.0f * (q1 * q2 + q0 * q3);
        result[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
        result[1][2] = 2.0f * (q2 * q3 - q0 * q1);
        result[2][0] = 2.0f * (q1 * q3 - q0 * q2);
        result[2][1] = 2.0f * (q2 * q3 + q0 * q1);
        result[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

        return result;
    }

    /**
     * @brief 获取四元数对应的轴角表示
     *
     * @return [Axis_X, Axis_Y, Axis_Z, Angle]^T, Angle单位rad
     * @note 选取0到PI范围内的等价最短旋转; 单位四元数正负号
     *       不会导致轴角结果跳到2PI附近.
     */
    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const
    {
        Class_Matrix_f32<4, 1> result;
        bool normalized_success = false;
        const Class_Quaternion_f32 normalized =
            Get_Normalization(&normalized_success);

        if (!normalized_success)
        {
            result[0][0] = 1.0f;
            return result;
        }

        float quaternion_w = normalized[0];
        float quaternion_x = normalized[1];
        float quaternion_y = normalized[2];
        float quaternion_z = normalized[3];

        // q与-q表示同一个旋转. 令实部非负可得到不超过PI的等价转角.
        if (quaternion_w < 0.0f)
        {
            quaternion_w = -quaternion_w;
            quaternion_x = -quaternion_x;
            quaternion_y = -quaternion_y;
            quaternion_z = -quaternion_z;
        }

        quaternion_w =
            std::max(-1.0f, std::min(1.0f, quaternion_w));
        result[3][0] = 2.0f * std::acos(quaternion_w);

        const float sin_half_angle =
            std::sqrt(std::max(
                0.0f,
                1.0f - quaternion_w * quaternion_w));

        if (sin_half_angle <= 1.0e-6f)
        {
            result[0][0] = 1.0f;
            result[1][0] = 0.0f;
            result[2][0] = 0.0f;
        }
        else
        {
            const float inverse_sin_half_angle =
                1.0f / sin_half_angle;
            result[0][0] =
                quaternion_x * inverse_sin_half_angle;
            result[1][0] =
                quaternion_y * inverse_sin_half_angle;
            result[2][0] =
                quaternion_z * inverse_sin_half_angle;
        }

        return result;
    }
};

/* Exported function declarations --------------------------------------------*/

namespace Namespace_ALG_Quaternion
{
inline Class_Quaternion_f32 Unit_Real()
{
    return Class_Quaternion_f32();
}

Class_Quaternion_f32 From_Euler_Angle(
    const Class_Matrix_f32<3, 1> &__Euler_Angle);
}

#endif /* ALG_QUATERNION_H */

/*----------------------------------------------------------------------------*/
