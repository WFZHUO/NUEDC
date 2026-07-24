/**
 * @file alg_quaternion.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 四元数计算支持库
 * @version 1.0
 * @date 2026-07-24 由达妙MC02 BSP四元数库适配
 */

#include "alg_quaternion.h"

/**
 * @brief 由Z-Y-X欧拉角构造四元数
 *
 * @param __Euler_Angle Yaw、Pitch、Roll, 单位rad
 */
Class_Quaternion_f32 Namespace_ALG_Quaternion::From_Euler_Angle(
    const Class_Matrix_f32<3, 1> &__Euler_Angle)
{
    const float half_yaw = __Euler_Angle[0][0] * 0.5f;
    const float half_pitch = __Euler_Angle[1][0] * 0.5f;
    const float half_roll = __Euler_Angle[2][0] * 0.5f;

    const float cy = std::cos(half_yaw);
    const float sy = std::sin(half_yaw);
    const float cp = std::cos(half_pitch);
    const float sp = std::sin(half_pitch);
    const float cr = std::cos(half_roll);
    const float sr = std::sin(half_roll);

    return Class_Quaternion_f32(
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr);
}

/*----------------------------------------------------------------------------*/
