#ifndef BSP_UART_H
#define BSP_UART_H

#include "app_config.h"
#include <stdint.h>

void BSP_UART_Init(void);
void BSP_UART_SendByte(uint8_t byte);
void BSP_UART_SendBuffer(const uint8_t *buf, uint16_t len);
void BSP_UART_RxIRQHandler(void);

#endif
