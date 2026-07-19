/**
 * @file bsp_power.h
 * @author WangFonzhuo
 * @brief 板载电源输出、电池电压检测与4S低压报警
 * @version 1.1
 * @date 2026-07-20 电赛主控
 */

#ifndef BSP_POWER_H
#define BSP_POWER_H

/* Includes ------------------------------------------------------------------*/

#include "drv_adc.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// 外部100K/10K分压还原系数
#define POWER_VOLTAGE_DIVIDER_RATIO ADC_VBAT_DIVIDER_RATIO

// 4S LiPo低电压报警阈值
#define POWER_4S_LOW_VOLTAGE_THRESHOLD 14.4f

// 6S LiPo低电压报警阈值
#define POWER_6S_LOW_VOLTAGE_THRESHOLD 21.6f

// 每10ms采样一次, 100点对应最近1s滑动平均
#define POWER_VOLTAGE_FILTER_SAMPLE_NUMBER 100U

// ADC DMA刚启动时忽略接近0V的无效读数
#define POWER_VALID_VOLTAGE_MIN 1.0f

// 低压报警歌曲响度
#define POWER_LOW_VOLTAGE_ALARM_LOUDNESS 1.0f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 板载电源控制与检测
 *
 * @note 包含两个VCC_OUT输出、一个5V输出、电池电压检测和电池低压报警
 */
class Class_Power
{
public:
    /**
     * @brief 初始化板载电源
     *
     * @param __hadc 电源电压检测使用的ADC
     * @param __VCC_OUT_1_Status VCC_OUT_1初始输出状态
     * @param __VCC_OUT_2_Status VCC_OUT_2初始输出状态
     * @param __5V_Status 5V初始输出状态
     * @param __Low_Voltage_Threshold 低电压报警阈值, 单位V
     * @param __Voltage_Divider_Ratio 外部分压还原系数
     */
    void Init(
        ADC_HandleTypeDef *__hadc,
        bool __VCC_OUT_1_Status = false,
        bool __VCC_OUT_2_Status = false,
        bool __5V_Status = false,
        float __Low_Voltage_Threshold = POWER_4S_LOW_VOLTAGE_THRESHOLD,
        float __Voltage_Divider_Ratio = POWER_VOLTAGE_DIVIDER_RATIO
    );

    /**
     * @brief 10ms定时更新回调
     *
     * @note 只读取电压、计算最近1s平均值并置位报警请求,
     *       不在定时器中断中播放歌曲
     */
    void TIM_10ms_Update_PeriodElapsedCallback();

    /**
     * @brief 主循环任务
     *
     * @note 在主循环中处理阻塞式See You Again低压报警
     */
    void Task_Loop();

    /**
     * @brief 清除滤波数据和低压报警锁存
     *
     * @note 不改变三个电源输出状态
     */
    void Reset_Voltage_Monitor();

    /**
     * @brief 关闭全部可控电源输出
     */
    void Disable_All_Output();

    /**
     * @brief 设置VCC_OUT_1输出
     */
    inline void Set_VCC_OUT_1(bool __Status);

    /**
     * @brief 设置VCC_OUT_2输出
     */
    inline void Set_VCC_OUT_2(bool __Status);

    /**
     * @brief 设置5V输出
     */
    inline void Set_DC5(bool __Status);

    /**
     * @brief 获取VCC_OUT_1输出状态
     */
    inline bool Get_VCC_OUT_1_Status() const;

    /**
     * @brief 获取VCC_OUT_2输出状态
     */
    inline bool Get_VCC_OUT_2_Status() const;

    /**
     * @brief 获取5V输出状态
     */
    inline bool Get_DC5_Status() const;

    /**
     * @brief 获取当前电源电压
     */
    inline float Get_Power_Voltage() const;

    /**
     * @brief 获取最近1s平均电源电压
     */
    inline float Get_Average_Power_Voltage() const;

    /**
     * @brief 获取低电压报警状态
     */
    inline bool Get_Low_Voltage_Status() const;

protected:
    // 绑定的ADC
    ADC_HandleTypeDef *ADC_Handler = nullptr;

    // 三路电源输出状态
    bool VCC_OUT_1_Status = false;
    bool VCC_OUT_2_Status = false;
    bool DC5_Status = false;

    // 电压参数
    float Low_Voltage_Threshold = POWER_4S_LOW_VOLTAGE_THRESHOLD;
    float Voltage_Divider_Ratio = POWER_VOLTAGE_DIVIDER_RATIO;

    // 当前电压和最近1s平均电压
    volatile float Power_Voltage = 0.0f;
    volatile float Average_Power_Voltage = 0.0f;

    // 1s滑动平均缓冲区
    float Voltage_Filter_Buffer[POWER_VOLTAGE_FILTER_SAMPLE_NUMBER] = {};
    float Voltage_Filter_Sum = 0.0f;
    uint16_t Voltage_Filter_Index = 0U;
    uint16_t Valid_Sample_Number = 0U;

    // 低压报警状态
    volatile bool Low_Voltage_Status = false;
    volatile bool Low_Voltage_Alarm_Pending = false;
};

/* Exported variables --------------------------------------------------------*/

// 全局板载电源对象
extern Class_Power BSP_Power;

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 设置VCC_OUT_1输出
 */
inline void Class_Power::Set_VCC_OUT_1(bool __Status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, __Status ? GPIO_PIN_SET : GPIO_PIN_RESET);

    VCC_OUT_1_Status = __Status;
}

/**
 * @brief 设置VCC_OUT_2输出
 */
inline void Class_Power::Set_VCC_OUT_2(bool __Status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, __Status ? GPIO_PIN_SET : GPIO_PIN_RESET);

    VCC_OUT_2_Status = __Status;
}

/**
 * @brief 设置5V输出
 */
inline void Class_Power::Set_DC5(bool __Status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, __Status ? GPIO_PIN_SET : GPIO_PIN_RESET);

    DC5_Status = __Status;
}

/**
 * @brief 获取VCC_OUT_1输出状态
 */
inline bool Class_Power::Get_VCC_OUT_1_Status() const
{
    return (VCC_OUT_1_Status);
}

/**
 * @brief 获取VCC_OUT_2输出状态
 */
inline bool Class_Power::Get_VCC_OUT_2_Status() const
{
    return (VCC_OUT_2_Status);
}

/**
 * @brief 获取5V输出状态
 */
inline bool Class_Power::Get_DC5_Status() const
{
    return (DC5_Status);
}

/**
 * @brief 获取当前电源电压
 */
inline float Class_Power::Get_Power_Voltage() const
{
    return (Power_Voltage);
}

/**
 * @brief 获取最近1s平均电源电压
 */
inline float Class_Power::Get_Average_Power_Voltage() const
{
    return (Average_Power_Voltage);
}

/**
 * @brief 获取低电压报警状态
 */
inline bool Class_Power::Get_Low_Voltage_Status() const
{
    return (Low_Voltage_Status);
}

#endif /* BSP_POWER_H */

/*----------------------------------------------------------------------------*/
