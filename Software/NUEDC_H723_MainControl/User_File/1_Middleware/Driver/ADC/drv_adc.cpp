/**
 * @file drv_adc.cpp
 * @author WangFonzhuo
 * @brief ADC通用接口
 * @version 1.0
 * @date 2026-07-19 电赛主控
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_adc.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// ADC管理对象
Struct_ADC_Manage_Object ADC1_Manage_Object = {};
Struct_ADC_Manage_Object ADC2_Manage_Object = {};
Struct_ADC_Manage_Object ADC3_Manage_Object = {};

// ADC DMA缓冲区放入RAM_D2, 避免DMA无法访问DTCM
__attribute__((section(".dma_buffer"), aligned(32)))
static uint16_t ADC1_Data_Buffer[ADC_BUFFER_SIZE];

__attribute__((section(".dma_buffer"), aligned(32)))
static uint16_t ADC2_Data_Buffer[ADC_BUFFER_SIZE];

__attribute__((section(".dma_buffer"), aligned(32)))
static uint16_t ADC3_Data_Buffer[ADC_BUFFER_SIZE];

/* Function prototypes -------------------------------------------------------*/

static Struct_ADC_Manage_Object *ADC_Get_Manage_Object(ADC_HandleTypeDef *hadc);
static uint16_t *ADC_Get_Data_Buffer(ADC_HandleTypeDef *hadc);
static float ADC_Get_Full_Scale_Value(ADC_HandleTypeDef *hadc);

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 获取ADC管理对象
 *
 * @param hadc ADC编号
 * @return Struct_ADC_Manage_Object* ADC管理对象
 */
static Struct_ADC_Manage_Object *ADC_Get_Manage_Object(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return (nullptr);
    }

    if (hadc->Instance == ADC1)
    {
        return (&ADC1_Manage_Object);
    }
#if defined(ADC2)
    else if (hadc->Instance == ADC2)
    {
        return (&ADC2_Manage_Object);
    }
#endif
#if defined(ADC3)
    else if (hadc->Instance == ADC3)
    {
        return (&ADC3_Manage_Object);
    }
#endif

    return (nullptr);
}

/**
 * @brief 获取ADC DMA缓冲区
 *
 * @param hadc ADC编号
 * @return uint16_t* ADC DMA缓冲区
 */
static uint16_t *ADC_Get_Data_Buffer(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return (nullptr);
    }

    if (hadc->Instance == ADC1)
    {
        return (ADC1_Data_Buffer);
    }
#if defined(ADC2)
    else if (hadc->Instance == ADC2)
    {
        return (ADC2_Data_Buffer);
    }
#endif
#if defined(ADC3)
    else if (hadc->Instance == ADC3)
    {
        return (ADC3_Data_Buffer);
    }
#endif

    return (nullptr);
}

/**
 * @brief 根据ADC分辨率获取满量程数字量
 *
 * @param hadc ADC编号
 * @return float ADC满量程数字量
 */
static float ADC_Get_Full_Scale_Value(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr)
    {
        return (0.0f);
    }

    switch (hadc->Init.Resolution)
    {
#if defined(ADC_RESOLUTION_16B)
        case ADC_RESOLUTION_16B:
            return (65535.0f);
#endif

#if defined(ADC_RESOLUTION_14B)
        case ADC_RESOLUTION_14B:
            return (16383.0f);
#endif

#if defined(ADC_RESOLUTION_12B)
        case ADC_RESOLUTION_12B:
            return (4095.0f);
#endif

#if defined(ADC_RESOLUTION_10B)
        case ADC_RESOLUTION_10B:
            return (1023.0f);
#endif

#if defined(ADC_RESOLUTION_8B)
        case ADC_RESOLUTION_8B:
            return (255.0f);
#endif

        default:
            return (0.0f);
    }
}

/**
 * @brief 初始化ADC并启动循环DMA采样
 *
 * @param hadc ADC编号
 * @param Sample_Number DMA缓冲区中的采样数量, 范围为1~ADC_BUFFER_SIZE
 * @return HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number)
{
    Struct_ADC_Manage_Object *manage_object = ADC_Get_Manage_Object(hadc);
    uint16_t *data_buffer = ADC_Get_Data_Buffer(hadc);

    if (hadc == nullptr ||
        manage_object == nullptr ||
        data_buffer == nullptr ||
        hadc->DMA_Handle == nullptr ||
        Sample_Number == 0U ||
        Sample_Number > ADC_BUFFER_SIZE)
    {
        return (HAL_ERROR);
    }

    // 防止重复初始化时ADC仍处于DMA采样状态
    (void)HAL_ADC_Stop_DMA(hadc);

    manage_object->ADC_Handler = hadc;
    manage_object->ADC_Data = data_buffer;
    manage_object->Sample_Number = Sample_Number;

    memset(data_buffer, 0, Sample_Number * sizeof(uint16_t));

    // H7 ADC启动前执行单端偏移校准
    HAL_StatusTypeDef status =
        HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    if (status != HAL_OK)
    {
        return (status);
    }

    status = HAL_ADC_Start_DMA(hadc,
                               reinterpret_cast<uint32_t *>(data_buffer),
                               Sample_Number);

    if (status == HAL_OK)
    {
        // 当前库不使用DMA半传输回调, 关闭HT中断以减少无效中断
        __HAL_DMA_DISABLE_IT(hadc->DMA_Handle, DMA_IT_HT);
    }

    return (status);
}

/**
 * @brief 停止ADC DMA采样
 *
 * @param hadc ADC编号
 * @return HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef ADC_Deinit(ADC_HandleTypeDef *hadc)
{
    Struct_ADC_Manage_Object *manage_object = ADC_Get_Manage_Object(hadc);

    if (hadc == nullptr || manage_object == nullptr)
    {
        return (HAL_ERROR);
    }

    HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(hadc);

    manage_object->ADC_Handler = nullptr;
    manage_object->ADC_Data = nullptr;
    manage_object->Sample_Number = 0U;

    return (status);
}

/**
 * @brief 获取DMA缓冲区中所有采样值的平均值
 *
 * @param hadc ADC编号
 * @return float ADC原始采样平均值
 */
float ADC_Get_Raw_Average(ADC_HandleTypeDef *hadc)
{
    Struct_ADC_Manage_Object *manage_object = ADC_Get_Manage_Object(hadc);

    if (manage_object == nullptr ||
        manage_object->ADC_Data == nullptr ||
        manage_object->Sample_Number == 0U)
    {
        return (0.0f);
    }

    uint32_t sum = 0U;

    for (uint16_t i = 0U; i < manage_object->Sample_Number; i++)
    {
        sum += manage_object->ADC_Data[i];
    }

    return (static_cast<float>(sum) /
            static_cast<float>(manage_object->Sample_Number));
}

/**
 * @brief 获取ADC引脚处的实际电压
 *
 * @param hadc ADC编号
 * @return float ADC引脚电压, 单位V
 */
float ADC_Get_Pin_Voltage(ADC_HandleTypeDef *hadc)
{
    float full_scale_value = ADC_Get_Full_Scale_Value(hadc);

    if (full_scale_value <= 0.0f)
    {
        return (0.0f);
    }

    return (ADC_Get_Raw_Average(hadc) *
            ADC_REFERENCE_VOLTAGE /
            full_scale_value);
}

/**
 * @brief 根据外部分压比还原被测电压
 *
 * @param hadc ADC编号
 * @param Voltage_Divider_Ratio 分压还原系数
 * @return float 被测电压, 单位V
 */
float ADC_Get_Source_Voltage(ADC_HandleTypeDef *hadc,
                             float Voltage_Divider_Ratio)
{
    if (Voltage_Divider_Ratio <= 0.0f)
    {
        return (0.0f);
    }

    return (ADC_Get_Pin_Voltage(hadc) * Voltage_Divider_Ratio);
}

/*----------------------------------------------------------------------------*/
