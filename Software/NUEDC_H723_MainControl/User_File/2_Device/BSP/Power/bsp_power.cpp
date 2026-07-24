/**
 * @file bsp_power.cpp
 * @author WangFonzhuo
 * @brief 板载电源输出、电池电压检测与4S低压报警
 * @version 1.1
 * @date 2026-07-20 电赛主控
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_power.h"
#include "bsp_BuzzerSongs.h"

#include <string.h>

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// 全局板载电源对象
Class_Power BSP_Power;

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化板载电源
 */
void Class_Power::Init(
    ADC_HandleTypeDef *__hadc,
    bool __VCC_OUT_1_Status,
    bool __VCC_OUT_2_Status,
    bool __5V_Status,
    float __Low_Voltage_Threshold,
    float __Voltage_Divider_Ratio)
{
    ADC_Handler = __hadc;

    Low_Voltage_Threshold =
        (__Low_Voltage_Threshold > 0.0f) ?
        __Low_Voltage_Threshold :
        POWER_4S_LOW_VOLTAGE_THRESHOLD;

    Voltage_Divider_Ratio =
        (__Voltage_Divider_Ratio > 0.0f) ?
        __Voltage_Divider_Ratio :
        POWER_VOLTAGE_DIVIDER_RATIO;

    Reset_Voltage_Monitor();

    Set_VCC_OUT_1(__VCC_OUT_1_Status);
    Set_VCC_OUT_2(__VCC_OUT_2_Status);
    Set_DC5(__5V_Status);
}

/**
 * @brief 10ms定时更新回调
 */
void Class_Power::TIM_10ms_Update_PeriodElapsedCallback()
{
    if (ADC_Handler == nullptr)
    {
        return;
    }

    float voltage = ADC_Get_Source_Voltage(ADC_Handler, Voltage_Divider_Ratio);

    /*
     * ADC DMA刚启动时缓冲区可能还没有有效数据。
     * 忽略接近0V的启动读数, 防止上电误报警。
     */
    if (voltage < POWER_VALID_VOLTAGE_MIN)
    {
        return;
    }

    Power_Voltage = voltage;

    /*
     * 前100次有效采样用于逐步填满滑动平均缓冲区。
     * 收满后, 每次移除最旧电压并加入最新电压。
     */
    if (Valid_Sample_Number < POWER_VOLTAGE_FILTER_SAMPLE_NUMBER)
    {
        Voltage_Filter_Buffer[Voltage_Filter_Index] = voltage;
        Voltage_Filter_Sum += voltage;

        Voltage_Filter_Index++;
        Valid_Sample_Number++;

        if (Voltage_Filter_Index >= POWER_VOLTAGE_FILTER_SAMPLE_NUMBER)
        {
            Voltage_Filter_Index = 0U;
        }

        Average_Power_Voltage = Voltage_Filter_Sum / static_cast<float>(Valid_Sample_Number);
    }
    else
    {
        Voltage_Filter_Sum -= Voltage_Filter_Buffer[Voltage_Filter_Index];

        Voltage_Filter_Buffer[Voltage_Filter_Index] = voltage;
        Voltage_Filter_Sum += voltage;

        Voltage_Filter_Index++;

        if (Voltage_Filter_Index >= POWER_VOLTAGE_FILTER_SAMPLE_NUMBER)
        {
            Voltage_Filter_Index = 0U;
        }

        Average_Power_Voltage = Voltage_Filter_Sum / static_cast<float>(POWER_VOLTAGE_FILTER_SAMPLE_NUMBER);
    }

    /*
     * 必须先形成完整的1s平均值后才允许报警。
     * 报警采用一次性锁存, 同一次上电期间只播放一次。
     */
    if ((Valid_Sample_Number >= POWER_VOLTAGE_FILTER_SAMPLE_NUMBER) &&
        (Low_Voltage_Status == false) &&
        (Average_Power_Voltage <= Low_Voltage_Threshold))
    {
        Low_Voltage_Status = true;
        Low_Voltage_Alarm_Pending = true;
    }
}

/**
 * @brief 主循环任务
 */
void Class_Power::Task_Loop()
{
    if (Low_Voltage_Alarm_Pending == false)
    {
        return;
    }

    /*
     * 先清除待处理标志, 再执行阻塞式歌曲播放。
     * 低压状态已经锁存, 播放期间不会再次产生报警请求。
     */
    Low_Voltage_Alarm_Pending = false;

    BuzzerSongs_Play_See_You_Again(
        POWER_LOW_VOLTAGE_ALARM_LOUDNESS
    );
}

/**
 * @brief 清除滤波数据和低压报警锁存
 */
void Class_Power::Reset_Voltage_Monitor()
{
    Power_Voltage = 0.0f;
    Average_Power_Voltage = 0.0f;

    memset(Voltage_Filter_Buffer, 0, sizeof(Voltage_Filter_Buffer));

    Voltage_Filter_Sum = 0.0f;
    Voltage_Filter_Index = 0U;
    Valid_Sample_Number = 0U;

    Low_Voltage_Status = false;
    Low_Voltage_Alarm_Pending = false;
}

/**
 * @brief 关闭全部可控电源输出
 */
void Class_Power::Disable_All_Output()
{
    Set_VCC_OUT_1(false);
    Set_VCC_OUT_2(false);
    Set_DC5(false);
}

/*----------------------------------------------------------------------------*/
