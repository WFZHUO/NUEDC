/**
 * @file bsp_bmi088_accel.h
 * @author WangFonzhuo
 * @brief BMI088加速度计原始数据解析
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

#ifndef BSP_BMI088_ACCEL_H
#define BSP_BMI088_ACCEL_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define BMI088_ACCEL_STANDARD_GRAVITY     (9.80665f)
#define BMI088_ACCEL_RANGE_G              (24.0f)
#define BMI088_ACCEL_SCALE_MPS2           \
    (BMI088_ACCEL_RANGE_G * BMI088_ACCEL_STANDARD_GRAVITY / 32768.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088加速度计数据
 */
class Class_BMI088_Accel
{
public:
    /**
     * @brief 解析连续六字节加速度数据
     *
     * @param __Buffer 从X轴LSB开始的数据
     * @param __Length 有效数据长度
     * @return bool 数据是否合法
     */
    bool Parse_Acceleration_Data(const uint8_t *__Buffer,
                                 uint16_t __Length);

    /**
     * @brief 解析连续两字节温度数据
     *
     * @param __Buffer 从温度MSB开始的数据
     * @param __Length 有效数据长度
     * @return bool 数据是否合法
     */
    bool Parse_Temperature_Data(const uint8_t *__Buffer,
                                uint16_t __Length);

    inline int16_t Get_Raw_X() const;
    inline int16_t Get_Raw_Y() const;
    inline int16_t Get_Raw_Z() const;
    inline float Get_X() const;
    inline float Get_Y() const;
    inline float Get_Z() const;
    inline int16_t Get_Raw_Temperature() const;
    inline float Get_Temperature() const;

protected:
    volatile int16_t Raw_X = 0;
    volatile int16_t Raw_Y = 0;
    volatile int16_t Raw_Z = 0;

    volatile float X = 0.0f;
    volatile float Y = 0.0f;
    volatile float Z = 0.0f;

    volatile int16_t Raw_Temperature = 0;
    volatile float Temperature = 0.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

inline int16_t Class_BMI088_Accel::Get_Raw_X() const
{
    return Raw_X;
}

inline int16_t Class_BMI088_Accel::Get_Raw_Y() const
{
    return Raw_Y;
}

inline int16_t Class_BMI088_Accel::Get_Raw_Z() const
{
    return Raw_Z;
}

inline float Class_BMI088_Accel::Get_X() const
{
    return X;
}

inline float Class_BMI088_Accel::Get_Y() const
{
    return Y;
}

inline float Class_BMI088_Accel::Get_Z() const
{
    return Z;
}

inline int16_t Class_BMI088_Accel::Get_Raw_Temperature() const
{
    return Raw_Temperature;
}

inline float Class_BMI088_Accel::Get_Temperature() const
{
    return Temperature;
}

#endif /* BSP_BMI088_ACCEL_H */

/*----------------------------------------------------------------------------*/
