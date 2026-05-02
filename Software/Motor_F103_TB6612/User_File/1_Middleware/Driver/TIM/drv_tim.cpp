/**
 * @file drv_tim.cpp
 * @author WangFonzhuo
 * @brief TIM通用接口
 * @version 1.0
 * @date 2025-12-30 26赛季定稿
 * @date 2026-04-18 27赛季
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_tim.h"

/* Macros --------------------------------------------------------------------*/

/* Types ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

// TIM定时器管理对象
Struct_TIM_Manage_Object TIM1_Manage_Object;
Struct_TIM_Manage_Object TIM2_Manage_Object;
Struct_TIM_Manage_Object TIM3_Manage_Object;
Struct_TIM_Manage_Object TIM4_Manage_Object;

/* Function prototypes -------------------------------------------------------*/

/* Function definitions ------------------------------------------------------*/

/**
 * @brief 初始化TIM定时器
 *
 * @param htim 定时器编号
 * @param Callback_Function 处理回调函数
 */
void TIM_Init(TIM_HandleTypeDef *htim, TIM_Call_Back Callback_Function)
{
    if (htim == nullptr)
    {
        return;
    }

    if (htim->Instance == TIM1)
    {
        TIM1_Manage_Object.TIM_Handler = htim;
        TIM1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM2)
    {
        TIM2_Manage_Object.TIM_Handler = htim;
        TIM2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Manage_Object.TIM_Handler = htim;
        TIM3_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM4)
    {
        TIM4_Manage_Object.TIM_Handler = htim;
        TIM4_Manage_Object.Callback_Function = Callback_Function;
    }
}

/**
 * @brief HAL库TIM定时器中断
 *
 * @param htim TIM编号
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 判断程序初始化完成
    if (init_finished == false)
    {
        return;
    }

    if (htim->Instance == TIM1)
    {
        if (TIM1_Manage_Object.Callback_Function != nullptr)
        {
            TIM1_Manage_Object.Callback_Function();
        }
    }
    else if (htim->Instance == TIM2)
    {
        if (TIM2_Manage_Object.Callback_Function != nullptr)
        {
            TIM2_Manage_Object.Callback_Function();
        }
    }
    else if (htim->Instance == TIM3)
    {
        if (TIM3_Manage_Object.Callback_Function != nullptr)
        {
            TIM3_Manage_Object.Callback_Function();
        }
    }
    else if (htim->Instance == TIM4)
    {
        if (TIM4_Manage_Object.Callback_Function != nullptr)
        {
            TIM4_Manage_Object.Callback_Function();
        }
    }
}

/*----------------------------------------------------------------------------*/
