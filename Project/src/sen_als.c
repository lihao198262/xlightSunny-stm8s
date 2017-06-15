#include <stm8s.h>
#include "ADC1Dev.h"

#include "sen_als.h"

// ALS sensor library
#define ALS_DATA_PORT                   GPIOB
#define ALS_DATA_PIN_ID                 GPIO_PIN_0

#define ALS_MA_NUM             20

bool als_ready = FALSE;
bool als_alive = FALSE;
u8 als_value;

// Moving average
u8 als_mvPtr = 0;
u8 als_mvData[ALS_MA_NUM] = {0};
u16 als_mvSum = 0;

bool als_checkData()
{
  u8 newData = als_read();
  if( newData > 100 ) return als_ready;
  
  als_alive = TRUE;
  if( newData != als_mvData[als_mvPtr] ) {
    als_mvSum += newData;
    als_mvSum -= als_mvData[als_mvPtr];
    als_mvData[als_mvPtr] = newData;
  }  
  als_mvPtr = (als_mvPtr + 1) % ALS_MA_NUM;
  if( !als_ready ) {
    als_ready = (als_mvPtr == 0);
  }
  
  if( als_ready ) als_value = als_mvSum / ALS_MA_NUM;
    
  return als_ready;
}


