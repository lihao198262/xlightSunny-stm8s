#include <stm8s.h>

#include "sen_als.h"

// ALS sensor library
#define ALS_DATA_PORT                   GPIOB
#define ALS_DATA_PIN_ID                 GPIO_PIN_0

void als_init()
{
  // Corresponding to ADC1_CHANNEL_0
  GPIO_Init(ALS_DATA_PORT, ALS_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
}

