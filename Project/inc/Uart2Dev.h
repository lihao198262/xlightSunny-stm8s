#ifndef _UART2_DEV_H_
#define _UART2_DEV_H_

#include "stm8s.h"
void Uart2SendByte(uint8_t data);
void uart2_config(uint32_t speed);
uint8_t Uart2SendString(uint8_t *pBuf);

#endif