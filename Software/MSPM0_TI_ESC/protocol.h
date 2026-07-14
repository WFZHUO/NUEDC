#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "app_config.h"
#include <stdint.h>

void Protocol_Init(void);
void Protocol_ParseByte(uint8_t byte);
void Protocol_1msTask(void);
void Protocol_SendFeedback(void);
uint32_t Protocol_GetLastCommandTimeMs(void);

#endif
