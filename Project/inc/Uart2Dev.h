#ifndef _UART2_DEV_H_
#define _UART2_DEV_H_

#include "stm8s.h"

void uart2_config(void);
uint8_t Uart2SendString(uint8_t *pBuf);

#endif