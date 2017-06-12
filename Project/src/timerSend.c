#include <stm8s.h>
#include "timerSend.h"
#include "_global.h"
#include "rf24l01.h"

uint16_t TIM4_TimingDelay = 0;
uint8_t TIM4_TimingDelayUs = 0;
uint16_t TIM4_SendDelay = 0;

uint8_t tim4_tried = 0;
uint8_t tim4_msgBuf[PLOAD_WIDTH];

void sleep_ms(uint16_t _ms) {
  TIM4_TimingDelay = _ms;

  // Interval = 8us*125 = 1ms
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_IMMEDIATE);
  TIM4_ARRPreloadConfig(ENABLE);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
  
  while(TIM4_TimingDelay > 0);
}

void sleep_8us(uint8_t _delay) {
  TIM4_TimingDelayUs = _delay;
  
  // Interval = 8us * _delay
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, _delay - 1);
  TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_IMMEDIATE);
  TIM4_ARRPreloadConfig(ENABLE);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
  
  while(TIM4_TimingDelayUs > 0);
}

void delaySend(uint8_t _base, uint16_t _ms, uint8_t *pMsg)
{
  tim4_tried = _base;
  memcpy(tim4_msgBuf, pMsg, PLOAD_WIDTH);
  TIM4_SendDelay = _ms;
  
  // Interval = 8us*125 = 1ms
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_IMMEDIATE);
  TIM4_ARRPreloadConfig(ENABLE);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
}

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{ 
  if(TIM4_TimingDelay > 0)
  {
    TIM4_TimingDelay--;
  }
  
  if(TIM4_TimingDelayUs > 0)
  {
    TIM4_TimingDelayUs = 0;
  }
  
  if(TIM4_SendDelay > 0)
  {
    TIM4_SendDelay--;
    if( TIM4_SendDelay == 0 ) {
      // Send message
      tim4_tried++;
      uint8_t rc = FastSendMessage(tim4_msgBuf, PLOAD_WIDTH);
      if (rc != 1) {
        if( tim4_tried <= gConfig.rptTimes ) {
          TIM4_SendDelay = TIM4_RETRY_INTERVAL_MS;
        }
      }
    }
  }
  
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  
  if(TIM4_TimingDelay == 0 && TIM4_TimingDelayUs == 0 && TIM4_SendDelay == 0)
  {
    TIM4_Cmd(DISABLE);
  }
}
