#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "app_config.h"
#include <stdint.h>

void BSP_Encoder_Init(void);
void BSP_Encoder_GPIOIRQHandler(void);
void BSP_Encoder_1msTask(void);
int32_t BSP_Encoder_GetCount(Motor_ID_t id);
int32_t BSP_Encoder_GetSpeedCPS(Motor_ID_t id);
void BSP_Encoder_Clear(Motor_ID_t id);

#endif
