#include <stm8s.h>

#include "sen_pir.h"

// PIR sensor library
#define PIR_DATA_PORT                   GPIOE
#define PIR_DATA_PIN_ID                 GPIO_PIN_5


void pir_init()
{
  GPIO_Init(PIR_DATA_PORT, PIR_DATA_PIN_ID, GPIO_MODE_IN_PU_NO_IT);
}

bool pir_read()
{
  BitStatus pir_st = GPIO_ReadInputPin(PIR_DATA_PORT, PIR_DATA_PIN_ID);
  return(pir_st != RESET);
}