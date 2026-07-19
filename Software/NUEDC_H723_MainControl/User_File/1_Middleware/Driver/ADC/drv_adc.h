/**
 * @file drv_adc.h
 * @author WangFonzhuo
 * @brief ADC通用接口
 * @version 1.0
 * @date 2026-07-19 电赛主控
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "adc.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// ADC DMA采样缓冲区长度, 单位为半字
#define ADC_BUFFER_SIZE 128U

// 当前板卡VDDA标称值
#define ADC_REFERENCE_VOLTAGE 3.3f

// 原理图中VBAT分压电阻: R86 = 100K, R87 = 10K
#define ADC_VBAT_R_UPPER_KOHM 100.0f
#define ADC_VBAT_R_LOWER_KOHM 10.0f
#define ADC_VBAT_DIVIDER_RATIO \
    ((ADC_VBAT_R_UPPER_KOHM + ADC_VBAT_R_LOWER_KOHM) / ADC_VBAT_R_LOWER_KOHM)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC采样管理结构体
 */
struct Struct_ADC_Manage_Object
{
    ADC_HandleTypeDef *ADC_Handler;

    // DMA持续更新的采样缓冲区
    volatile uint16_t *ADC_Data;

    // 当前实际使用的采样数量
    uint16_t Sample_Number;
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_ADC_Manage_Object ADC1_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC2_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC3_Manage_Object;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 初始化ADC并启动循环DMA采样
 *
 * @param hadc ADC编号
 * @param Sample_Number DMA缓冲区中的采样数量, 范围为1~ADC_BUFFER_SIZE
 * @return HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number);

/**
 * @brief 停止ADC DMA采样
 *
 * @param hadc ADC编号
 * @return HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef ADC_Deinit(ADC_HandleTypeDef *hadc);

/**
 * @brief 获取DMA缓冲区中所有采样值的平均值
 *
 * @param hadc ADC编号
 * @return float ADC原始采样平均值
 */
float ADC_Get_Raw_Average(ADC_HandleTypeDef *hadc);

/**
 * @brief 获取ADC引脚处的实际电压
 *
 * @param hadc ADC编号
 * @return float ADC引脚电压, 单位V
 */
float ADC_Get_Pin_Voltage(ADC_HandleTypeDef *hadc);

/**
 * @brief 根据外部分压比还原被测电压
 *
 * @param hadc ADC编号
 * @param Voltage_Divider_Ratio 分压还原系数, 例如100K/10K分压时为11
 * @return float 被测电压, 单位V
 */
float ADC_Get_Source_Voltage(ADC_HandleTypeDef *hadc, float Voltage_Divider_Ratio);

#ifdef __cplusplus
}
#endif

#endif /* DRV_ADC_H */

/*----------------------------------------------------------------------------*/
