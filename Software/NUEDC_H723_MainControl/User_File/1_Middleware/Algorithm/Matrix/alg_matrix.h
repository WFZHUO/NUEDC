/**
 * @file alg_matrix.h
 * @author yssickjgd (1345578933@qq.com)
 * @author WangFonzhuo
 * @brief 固定尺寸单精度矩阵运算
 * @version 1.0
 * @date 2026-07-24 由达妙MC02 BSP矩阵库适配
 *
 * @copyright USTC-RoboWalker (c) 2025
 */

#ifndef ALG_MATRIX_H
#define ALG_MATRIX_H

/* Includes ------------------------------------------------------------------*/

#include <algorithm>
#include <cmath>
#include <cstring>
#include <type_traits>

/* Exported variables --------------------------------------------------------*/

extern float Matrix_Compare_Epsilon;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 固定尺寸单精度矩阵
 *
 * @note 数据完全静态分配, 不使用堆内存.
 */
template<int Row, int Column>
class Class_Matrix_f32
{
public:
    float Data[Row * Column] = {};

    Class_Matrix_f32() = default;

    explicit Class_Matrix_f32(const float *__Data)
    {
        if (__Data != nullptr)
        {
            std::memcpy(Data,
                        __Data,
                        sizeof(float) * Row * Column);
        }
    }

    inline float *operator[](int __Index)
    {
        return &Data[__Index * Column];
    }

    inline const float *operator[](int __Index) const
    {
        return &Data[__Index * Column];
    }

    inline Class_Matrix_f32<Row, Column> operator+(
        const Class_Matrix_f32<Row, Column> &__Matrix) const
    {
        Class_Matrix_f32<Row, Column> result;

        for (int i = 0; i < Row * Column; i++)
        {
            result.Data[i] = Data[i] + __Matrix.Data[i];
        }

        return result;
    }

    inline Class_Matrix_f32<Row, Column> operator-(
        const Class_Matrix_f32<Row, Column> &__Matrix) const
    {
        Class_Matrix_f32<Row, Column> result;

        for (int i = 0; i < Row * Column; i++)
        {
            result.Data[i] = Data[i] - __Matrix.Data[i];
        }

        return result;
    }

    inline Class_Matrix_f32<Row, Column> operator-() const
    {
        Class_Matrix_f32<Row, Column> result;

        for (int i = 0; i < Row * Column; i++)
        {
            result.Data[i] = -Data[i];
        }

        return result;
    }

    inline Class_Matrix_f32<Row, Column> operator*(
        float __Value) const
    {
        Class_Matrix_f32<Row, Column> result;

        for (int i = 0; i < Row * Column; i++)
        {
            result.Data[i] = Data[i] * __Value;
        }

        return result;
    }

    inline friend Class_Matrix_f32<Row, Column> operator*(
        float __Value,
        const Class_Matrix_f32<Row, Column> &__Matrix)
    {
        return __Matrix * __Value;
    }

    template<int Result_Column>
    inline Class_Matrix_f32<Row, Result_Column> operator*(
        const Class_Matrix_f32<Column, Result_Column> &__Matrix) const
    {
        Class_Matrix_f32<Row, Result_Column> result;

        for (int i = 0; i < Row; i++)
        {
            for (int k = 0; k < Column; k++)
            {
                const float left = Data[i * Column + k];

                for (int j = 0; j < Result_Column; j++)
                {
                    result.Data[i * Result_Column + j] +=
                        left * __Matrix.Data[k * Result_Column + j];
                }
            }
        }

        return result;
    }

    inline Class_Matrix_f32<Column, Row> Get_Transpose() const
    {
        Class_Matrix_f32<Column, Row> result;

        for (int i = 0; i < Row; i++)
        {
            for (int j = 0; j < Column; j++)
            {
                result[j][i] = Data[i * Column + j];
            }
        }

        return result;
    }

    inline float Get_Trace() const
    {
        float result = 0.0f;
        const int size = std::min(Row, Column);

        for (int i = 0; i < size; i++)
        {
            result += Data[i * Column + i];
        }

        return result;
    }

    template<int R = Row, int C = Column>
    inline std::enable_if_t<R == C, Class_Matrix_f32<R, R>>
    Get_Inverse(bool *__Success = nullptr) const
    {
        Class_Matrix_f32<R, 2 * R> extended;

        for (int i = 0; i < R; i++)
        {
            std::memcpy(&extended.Data[i * 2 * R],
                        &Data[i * R],
                        sizeof(float) * R);
            extended[i][i + R] = 1.0f;
        }

        for (int i = 0; i < R; i++)
        {
            int pivot_row = i;
            float pivot_absolute = std::fabs(extended[i][i]);

            for (int row = i + 1; row < R; row++)
            {
                const float candidate =
                    std::fabs(extended[row][i]);

                if (candidate > pivot_absolute)
                {
                    pivot_absolute = candidate;
                    pivot_row = row;
                }
            }

            if ((!std::isfinite(pivot_absolute)) ||
                (pivot_absolute <= Matrix_Compare_Epsilon))
            {
                if (__Success != nullptr)
                {
                    *__Success = false;
                }

                return Class_Matrix_f32<R, R>();
            }

            if (pivot_row != i)
            {
                for (int column = 0; column < 2 * R; column++)
                {
                    std::swap(extended[i][column],
                              extended[pivot_row][column]);
                }
            }

            const float inverse_pivot = 1.0f / extended[i][i];

            for (int column = 0; column < 2 * R; column++)
            {
                extended[i][column] *= inverse_pivot;
            }

            for (int row = 0; row < R; row++)
            {
                if (row == i)
                {
                    continue;
                }

                const float factor = extended[row][i];

                for (int column = 0; column < 2 * R; column++)
                {
                    extended[row][column] -=
                        factor * extended[i][column];
                }
            }
        }

        Class_Matrix_f32<R, R> result;

        for (int i = 0; i < R; i++)
        {
            std::memcpy(&result.Data[i * R],
                        &extended.Data[i * 2 * R + R],
                        sizeof(float) * R);
        }

        if (__Success != nullptr)
        {
            *__Success = true;
        }

        return result;
    }

    template<int R = Row, int C = Column>
    inline std::enable_if_t<C == 1, float> Get_Modulus() const
    {
        float norm_squared = 0.0f;

        for (int i = 0; i < R; i++)
        {
            norm_squared += Data[i] * Data[i];
        }

        return std::sqrt(norm_squared);
    }

    template<int R = Row, int C = Column>
    inline std::enable_if_t<C == 1, Class_Matrix_f32<R, 1>>
    Get_Normalization(bool *__Success = nullptr) const
    {
        Class_Matrix_f32<R, 1> result;
        const float modulus = Get_Modulus<R, C>();

        if ((!std::isfinite(modulus)) ||
            (modulus <= Matrix_Compare_Epsilon))
        {
            if (__Success != nullptr)
            {
                *__Success = false;
            }

            return result;
        }

        const float inverse_modulus = 1.0f / modulus;

        for (int i = 0; i < R; i++)
        {
            result.Data[i] = Data[i] * inverse_modulus;
        }

        if (__Success != nullptr)
        {
            *__Success = true;
        }

        return result;
    }
};

/* Exported function declarations --------------------------------------------*/

namespace Namespace_ALG_Matrix
{
template<int Row, int Column>
inline Class_Matrix_f32<Row, Column> Zero()
{
    return Class_Matrix_f32<Row, Column>();
}

template<int Row, int Column>
inline Class_Matrix_f32<Row, Column> Identity()
{
    Class_Matrix_f32<Row, Column> result;
    const int size = std::min(Row, Column);

    for (int i = 0; i < size; i++)
    {
        result[i][i] = 1.0f;
    }

    return result;
}

template<int Row>
inline float Operator_Dot(
    const Class_Matrix_f32<Row, 1> &__Vector_1,
    const Class_Matrix_f32<Row, 1> &__Vector_2)
{
    float result = 0.0f;

    for (int i = 0; i < Row; i++)
    {
        result += __Vector_1[i][0] * __Vector_2[i][0];
    }

    return result;
}
}

#endif /* ALG_MATRIX_H */

/*----------------------------------------------------------------------------*/
