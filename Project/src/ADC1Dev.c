#include "ADC1Dev.h"
#include "_global.h"

// ToDO: 
// 1. Change to Interupt mode
// 2. Average values and discard extreme samples

// ALS sensor library
#define ANI_DATA_PORT                   GPIOB
#define ALS_DATA_PIN_ID                 GPIO_PIN_0
#define MIC_DATA_PIN_ID                 GPIO_PIN_1

#ifdef EN_SENSOR_ALS || EN_SENSOR_MIC
void ADC1_Config()
{
  ADC1_DeInit();

#ifdef EN_SENSOR_MIC
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_1, ADC1_PRESSEL_FCPU_D4,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL1, DISABLE);
  ADC1_ScanModeCmd(ENABLE);
#else  
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_0, ADC1_PRESSEL_FCPU_D4,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0, DISABLE);
  ADC1_Cmd(ENABLE);
#endif
  
  //ADC1_ITConfig(ADC1_IT_AWS0, ENABLE);
  
  ADC1_StartConversion();
}

void ADC1_PinInit()
{
#ifdef EN_SENSOR_ALS
  // Corresponding to ADC1_CHANNEL_0
  GPIO_Init(ANI_DATA_PORT, ALS_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
#endif  

#ifdef EN_SENSOR_MIC
  // Corresponding to ADC1_CHANNEL_1
  GPIO_Init(ANI_DATA_PORT, MIC_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
#endif  
}
#endif

#ifdef EN_SENSOR_ALS
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
#endif

#ifdef EN_SENSOR_MIC
uint16_t mic_read()
{
  uint16_t level;
  
  //level = ADC1_GetBufferValue(0x00);
  level = ADC1_GetBufferValue(0x01);
        
  return level;
}
#endif