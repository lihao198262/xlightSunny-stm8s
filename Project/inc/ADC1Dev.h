#ifndef _ADC1_DEV_H_
#define _ADC1_DEV_H_

#include "stm8s.h"

void ADC1_Config();
void ADC1_PinInit();

uint8_t als_read();
uint16_t mic_read();

#endif