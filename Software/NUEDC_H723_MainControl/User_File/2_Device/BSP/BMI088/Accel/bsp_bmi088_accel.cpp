/**
 * @file bsp_bmi088_accel.cpp
 * @author WangFonzhuo
 * @brief BMI088加速度计原始数据解析
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 解析连续六字节加速度数据
 *
 * @param __Buffer 从X轴LSB开始的数据
 * @param __Length 有效数据长度
 * @return bool 数据是否合法
 */
bool Class_BMI088_Accel::Parse_Acceleration_Data(const uint8_t *__Buffer,
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

    X = static_cast<float>(Raw_X) * BMI088_ACCEL_SCALE_MPS2;
    Y = static_cast<float>(Raw_Y) * BMI088_ACCEL_SCALE_MPS2;
    Z = static_cast<float>(Raw_Z) * BMI088_ACCEL_SCALE_MPS2;

    return true;
}

/**
 * @brief 解析连续两字节温度数据
 *
 * @param __Buffer 从温度MSB开始的数据
 * @param __Length 有效数据长度
 * @return bool 数据是否合法
 */
bool Class_BMI088_Accel::Parse_Temperature_Data(const uint8_t *__Buffer,
                                                 uint16_t __Length)
{
    if ((__Buffer == nullptr) || (__Length < 2U) || (__Buffer[0] == 0x80U))
    {
        return false;
    }

    int16_t raw_temperature =
        static_cast<int16_t>((static_cast<uint16_t>(__Buffer[0]) << 3U) |
                             (static_cast<uint16_t>(__Buffer[1]) >> 5U));

    if (raw_temperature > 1023)
    {
        raw_temperature -= 2048;
    }

    Raw_Temperature = raw_temperature;
    Temperature = 23.0f + static_cast<float>(Raw_Temperature) * 0.125f;

    return true;
}

/*----------------------------------------------------------------------------*/
