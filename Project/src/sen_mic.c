#include <stm8s.h>

#include "sen_mic.h"

// MIC sensor library
#define MIC_DATA_PORT                   GPIOB
#define MIC_DATA_PIN_ID                 GPIO_PIN_1

void mic_init()
{
  // Corresponding to ADC1_CHANNEL_2
  GPIO_Init(MIC_DATA_PORT, MIC_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
}

