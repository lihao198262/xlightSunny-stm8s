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

// Convert to level: [0..100]
uint8_t als_read()
{
  // From [0..1023]
  uint16_t adc_value = ADC1_GetConversionValue();
  // Scale down to [0..100]
  uint8_t level = adc_value * 100 / 1023;
  return level;
}