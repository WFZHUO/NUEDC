/**
 * @file bsp_bmi088_gyro_register.h
 * @author WangFonzhuo
 * @brief BMI088陀螺仪寄存器定义
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

#ifndef BSP_BMI088_GYRO_REGISTER_H
#define BSP_BMI088_GYRO_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define BMI088_GYRO_READ_MASK             (0x80U)
#define BMI088_GYRO_CHIP_ID_VALUE         (0x0FU)
#define BMI088_GYRO_SOFT_RESET_VALUE      (0xB6U)
#define BMI088_GYRO_RANGE_2000DPS         (0x00U)
#define BMI088_GYRO_BANDWIDTH_2000_230HZ  (0x81U)
#define BMI088_GYRO_DATA_READY_ENABLE     (0x80U)
#define BMI088_GYRO_INT3_OUTPUT_HIGH      (0x0CU)
#define BMI088_GYRO_INT3_DATA_READY       (0x01U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088陀螺仪寄存器地址
 */
enum Enum_BMI088_Gyro_Register : uint8_t
{
    BMI088_Gyro_Register_CHIP_ID = 0x00U,
    BMI088_Gyro_Register_X_LSB = 0x02U,
    BMI088_Gyro_Register_RANGE = 0x0FU,
    BMI088_Gyro_Register_BANDWIDTH = 0x10U,
    BMI088_Gyro_Register_SOFT_RESET = 0x14U,
    BMI088_Gyro_Register_INT_CTRL = 0x15U,
    BMI088_Gyro_Register_INT3_INT4_IO_CONF = 0x16U,
    BMI088_Gyro_Register_INT3_INT4_IO_MAP = 0x18U,
};

/**
 * @brief BMI088陀螺仪初始化寄存器配置
 */
struct Struct_BMI088_Gyro_Register_Config
{
    uint8_t Address;
    uint8_t Value;
    uint16_t Delay_ms;
};

/* Exported variables --------------------------------------------------------*/

extern const Struct_BMI088_Gyro_Register_Config BMI088_Gyro_Init_Table[];
extern const uint32_t BMI088_Gyro_Init_Table_Length;

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

#endif /* BSP_BMI088_GYRO_REGISTER_H */

/*----------------------------------------------------------------------------*/
