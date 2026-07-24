/**
 * @file bsp_bmi088_accel_register.cpp
 * @author WangFonzhuo
 * @brief BMI088加速度计初始化寄存器配置
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel_register.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

const Struct_BMI088_Accel_Register_Config BMI088_Accel_Init_Table[] =
{
    {BMI088_Accel_Register_POWER_CTRL, BMI088_ACCEL_POWER_ON_VALUE, 2U},
    {BMI088_Accel_Register_POWER_CONF, BMI088_ACCEL_POWER_ACTIVE_VALUE, 1U},
    {BMI088_Accel_Register_ACCEL_CONF, BMI088_ACCEL_CONF_1600HZ_NORMAL, 1U},
    {BMI088_Accel_Register_ACCEL_RANGE, BMI088_ACCEL_RANGE_24G, 1U},
    {BMI088_Accel_Register_INT1_IO_CTRL, BMI088_ACCEL_INT1_OUTPUT_HIGH, 1U},
    {BMI088_Accel_Register_INT_MAP_DATA, BMI088_ACCEL_INT1_DATA_READY, 1U},
};

const uint32_t BMI088_Accel_Init_Table_Length =
    sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]);

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
