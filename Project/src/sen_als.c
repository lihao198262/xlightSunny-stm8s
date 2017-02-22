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
  uint8_t level;
  
  // Wait convert finished
  while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
  // Get value
  uint16_t adc_value = ADC1_GetConversionValue();
  // Clear flag
  ADC1_ClearFlag(ADC1_FLAG_EOC);
  // Start next conversion
  ADC1_StartConversion();
  
  // [0..1023], reversed scale down to [100..0]
  if( adc_value >= 1000 ) {
    level = 0;
  } else {
    level = 100 - adc_value / 10;
  }
  return level;
}