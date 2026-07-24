/**
 * @file bsp_bmi088_accel_register.h
 * @author WangFonzhuo
 * @brief BMI088加速度计寄存器定义
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

#ifndef BSP_BMI088_ACCEL_REGISTER_H
#define BSP_BMI088_ACCEL_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define BMI088_ACCEL_READ_MASK            (0x80U)
#define BMI088_ACCEL_CHIP_ID_VALUE        (0x1EU)
#define BMI088_ACCEL_SOFT_RESET_VALUE     (0xB6U)
#define BMI088_ACCEL_POWER_ON_VALUE       (0x04U)
#define BMI088_ACCEL_POWER_ACTIVE_VALUE   (0x00U)
#define BMI088_ACCEL_CONF_1600HZ_NORMAL   (0xACU)
#define BMI088_ACCEL_RANGE_24G            (0x03U)
#define BMI088_ACCEL_INT1_OUTPUT_HIGH     (0x08U)
#define BMI088_ACCEL_INT1_DATA_READY      (0x04U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088加速度计寄存器地址
 */
enum Enum_BMI088_Accel_Register : uint8_t
{
    BMI088_Accel_Register_CHIP_ID = 0x00U,
    BMI088_Accel_Register_X_LSB = 0x12U,
    BMI088_Accel_Register_TEMPERATURE_MSB = 0x22U,
    BMI088_Accel_Register_ACCEL_CONF = 0x40U,
    BMI088_Accel_Register_ACCEL_RANGE = 0x41U,
    BMI088_Accel_Register_INT1_IO_CTRL = 0x53U,
    BMI088_Accel_Register_INT_MAP_DATA = 0x58U,
    BMI088_Accel_Register_POWER_CONF = 0x7CU,
    BMI088_Accel_Register_POWER_CTRL = 0x7DU,
    BMI088_Accel_Register_SOFT_RESET = 0x7EU,
};

/**
 * @brief BMI088加速度计初始化寄存器配置
 */
struct Struct_BMI088_Accel_Register_Config
{
    uint8_t Address;
    uint8_t Value;
    uint16_t Delay_ms;
};

/* Exported variables --------------------------------------------------------*/

extern const Struct_BMI088_Accel_Register_Config BMI088_Accel_Init_Table[];
extern const uint32_t BMI088_Accel_Init_Table_Length;

/* Exported function prototypes ----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

#endif /* BSP_BMI088_ACCEL_REGISTER_H */

/*----------------------------------------------------------------------------*/
