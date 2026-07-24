/**
 * @file bsp_bmi088_gyro_register.cpp
 * @author WangFonzhuo
 * @brief BMI088陀螺仪初始化寄存器配置
 * @version 1.0
 * @date 2026-07-23 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro_register.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

const Struct_BMI088_Gyro_Register_Config BMI088_Gyro_Init_Table[] =
{
    {BMI088_Gyro_Register_RANGE, BMI088_GYRO_RANGE_2000DPS, 1U},
    {BMI088_Gyro_Register_BANDWIDTH, BMI088_GYRO_BANDWIDTH_2000_230HZ, 1U},
    {BMI088_Gyro_Register_INT_CTRL, BMI088_GYRO_DATA_READY_ENABLE, 1U},
    {BMI088_Gyro_Register_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_OUTPUT_HIGH, 1U},
    {BMI088_Gyro_Register_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DATA_READY, 1U},
};

const uint32_t BMI088_Gyro_Init_Table_Length =
    sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]);

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
