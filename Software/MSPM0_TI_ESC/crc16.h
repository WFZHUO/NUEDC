#ifndef CRC16_H
#define CRC16_H
#include <stdint.h>
#include <stddef.h>
uint16_t CRC16_Modbus(const uint8_t *data, uint16_t length);
#endif
