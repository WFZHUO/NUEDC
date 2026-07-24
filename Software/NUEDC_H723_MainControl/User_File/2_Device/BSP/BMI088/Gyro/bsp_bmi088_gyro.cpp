/**
 * @file bsp_bmi088_gyro.cpp
 * @author WangFonzhuo
 * @brief BMI088陀螺仪原始数据解析
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 解析连续六字节角速度数据
 *
 * @param __Buffer 从X轴LSB开始的数据
 * @param __Length 有效数据长度
 * @return bool 数据是否合法
 */
bool Class_BMI088_Gyro::Parse_Data(const uint8_t *__Buffer,
                                    uint16_t __Length)
{
    if ((__Buffer == nullptr) || (__Length < 6U))
    {
        return false;
    }

    Raw_X = static_cast<int16_t>((static_cast<uint16_t>(__Buffer[1]) << 8U) |
                                 static_cast<uint16_t>(__Buffer[0]));
    Raw_Y = static_cast<int16_t>((static_cast<uint16_t>(__Buffer[3]) << 8U) |
                                 static_cast<uint16_t>(__Buffer[2]));
    Raw_Z = static_cast<int16_t>((static_cast<uint16_t>(__Buffer[5]) << 8U) |
                                 static_cast<uint16_t>(__Buffer[4]));

    X = static_cast<float>(Raw_X) * BMI088_GYRO_SCALE_RADPS;
    Y = static_cast<float>(Raw_Y) * BMI088_GYRO_SCALE_RADPS;
    Z = static_cast<float>(Raw_Z) * BMI088_GYRO_SCALE_RADPS;

    return true;
}

/*----------------------------------------------------------------------------*/
