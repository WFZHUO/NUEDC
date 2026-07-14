#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "app_config.h"
#include <stdint.h>

void BSP_Motor_Init(void);
void BSP_Motor_SetPWM(Motor_ID_t id, int16_t pwm);
void BSP_Motor_SetPWM2(int16_t pwm_a, int16_t pwm_b);
void BSP_Motor_StopAll(void);
int16_t BSP_Motor_GetPWM(Motor_ID_t id);

#endif
