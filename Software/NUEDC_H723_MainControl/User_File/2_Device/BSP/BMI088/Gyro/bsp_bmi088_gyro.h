/**
 * @file bsp_bmi088_gyro.h
 * @author WangFonzhuo
 * @brief BMI088陀螺仪原始数据解析
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

#ifndef BSP_BMI088_GYRO_H
#define BSP_BMI088_GYRO_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define BMI088_GYRO_PI                     (3.14159265358979323846f)
#define BMI088_GYRO_RANGE_DPS              (2000.0f)
#define BMI088_GYRO_SCALE_RADPS            \
    (BMI088_GYRO_RANGE_DPS * BMI088_GYRO_PI / 180.0f / 32768.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088陀螺仪数据
 */
class Class_BMI088_Gyro
{
public:
    /**
     * @brief 解析连续六字节角速度数据
     *
     * @param __Buffer 从X轴LSB开始的数据
     * @param __Length 有效数据长度
     * @return bool 数据是否合法
     */
    bool Parse_Data(const uint8_t *__Buffer,
                    uint16_t __Length);

    inline int16_t Get_Raw_X() const;
    inline int16_t Get_Raw_Y() const;
    inline int16_t Get_Raw_Z() const;
    inline float Get_X() const;
    inline float Get_Y() const;
    inline float Get_Z() const;

protected:
    volatile int16_t Raw_X = 0;
    volatile int16_t Raw_Y = 0;
    volatile int16_t Raw_Z = 0;

    volatile float X = 0.0f;
    volatile float Y = 0.0f;
    volatile float Z = 0.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

inline int16_t Class_BMI088_Gyro::Get_Raw_X() const
{
    return Raw_X;
}

inline int16_t Class_BMI088_Gyro::Get_Raw_Y() const
{
    return Raw_Y;
}

inline int16_t Class_BMI088_Gyro::Get_Raw_Z() const
{
    return Raw_Z;
}

inline float Class_BMI088_Gyro::Get_X() const
{
    return X;
}

inline float Class_BMI088_Gyro::Get_Y() const
{
    return Y;
}

inline float Class_BMI088_Gyro::Get_Z() const
{
    return Z;
}

#endif /* BSP_BMI088_GYRO_H */

/*----------------------------------------------------------------------------*/
