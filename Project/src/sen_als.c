#include <stm8s.h>

#include "sen_als.h"

// ALS sensor library
#define ALS_DATA_PORT                   GPIOB
#define ALS_DATA_PIN_ID                 GPIO_PIN_0

void als_init()
{
  GPIO_Init(ALS_DATA_PORT, ALS_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
}

uint16_t als_read()
{
  BitStatus pir_st = GPIO_ReadInputPin(ALS_DATA_PORT, ALS_DATA_PIN_ID);
  return (bool)pir_st;
}