/*;*******************************************************************************
; FILENAME		: LightPwmDrv.c
; PURPOSE		: PWM-based Lighting Driver
; REVISION		: 1.0
; SYSTEM CLOCK	        : 16MHz
; MCU type              : STM8S105K4
;*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "LightPwmDrv.h"
#include "_global.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// Channel 1
#define WARM_LIGHT_PWM_PIN_PORT         GPIOD
#define WARM_LIGHT_PWM_PIN_ID           GPIO_PIN_3

// Channel 2
#define COLD_LIGHT_PWM_PIN_PORT         GPIOD
#define COLD_LIGHT_PWM_PIN_ID           GPIO_PIN_4

#define TIM2_PWM_PERIOD         199
#define TIM2_PWM_PULSE          200

/**
  * @brief  Configure peripherals GPIO for WARM/COLD PWM   
  * @param  None
  * @retval None
  */
static void TIM2WarmColdLightPwm_ClkGpioConfig(void)
{
  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

  /* TIM2 Channel1 configuration: PB2 */
  GPIO_Init(COLD_LIGHT_PWM_PIN_PORT, COLD_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(WARM_LIGHT_PWM_PIN_PORT, WARM_LIGHT_PWM_PIN_ID, GPIO_MODE_OUT_PP_LOW_FAST);
}

/**
  * @brief  Configure TIM2 peripheral   
  * @param  None
  * @retval None
  */
static void TIM2PWMFunction_Config(void)
{
   TIM2_DeInit();
  /* TIM2 configuration:
     - TIM2 counter is clocked by HSI div 8 2M
      so the TIM2 counter clock used is HSI / 1 = 16M / 1 = 16 MHz
    TIM2 Channel1 output frequency = TIM2CLK / (TIM2 Prescaler * (TIM2_PERIOD + 1))
    = 16M / (8 * 200) = 10K Hz //TIM2_Prescaler_8; TIM2_PWM_PERIOD = 199
  */
  /* Time Base configuration */
  TIM2_TimeBaseInit(TIM2_PRESCALER_8, TIM2_PWM_PERIOD);
  //TIM2_ETRClockMode2Config(TIM2_ExtTRGPSC_DIV4, TIM2_ExtTRGPolarity_NonInverted, 0);

  /* Channel 1 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_PWM_PULSE, TIM2_OCPOLARITY_HIGH);
  TIM2_OC1PreloadConfig(ENABLE);

  /* Channel 2 configuration in PWM1 mode */
  /* TIM2 channel Duty cycle is 100 * (TIM2_PERIOD + 1 - TIM2_PULSE) / (TIM2_PWM_PERIOD + 1) = 100 * 4/8 = 50 % */
  TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_PWM_PULSE, TIM2_OCPOLARITY_HIGH);
  TIM2_OC2PreloadConfig(ENABLE);

  /* TIM2 Main Output Enable */
  TIM2_ARRPreloadConfig(ENABLE);

  /* TIM2 counter enable */
  TIM2_Cmd(ENABLE);
}

void initTim2PWMFunction (void)
{
  TIM2WarmColdLightPwm_ClkGpioConfig ();
  TIM2PWMFunction_Config ();
}

void regulateColdLightPulseWidth (unsigned char ucPercent)
{
  // uint16_t pulseWidth;
  if (ucPercent >= 100)
    ucPercent = 100;
  //pulseWidth = 100 * (TIM2_PWM_PERIOD + 1) / ucPercent;
    
  //TIM2_CtrlPWMOutputs(DISABLE);
  //TIM2_Cmd(DISABLE);
  TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, ucPercent * 2, TIM2_OCPOLARITY_HIGH);
  //TIM2_OC1PreloadConfig(ENABLE);
  //TIM2_CtrlPWMOutputs(ENABLE);
  //TIM2_Cmd(ENABLE);
}

void regulateWarmLightPulseWidth (unsigned char ucPercent)
{
  //uint16_t pulseWidth;
  if (ucPercent >= 100)
    ucPercent = 100;
  
  //pulseWidth = ucPercent * 2;
  //pulseWidth = 100 * (TIM2_PWM_PERIOD + 1) / pulseWidth;
  
  
  //TIM2_CtrlPWMOutputs(DISABLE);
  //TIM2_Cmd(DISABLE);
  TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, ucPercent * 2, TIM2_OCPOLARITY_HIGH);  
  //TIM2_OC2PreloadConfig(ENABLE);
  //TIM2_CtrlPWMOutputs(ENABLE);
  //TIM2_Cmd(ENABLE);
}

void driveColdWarmLightPwm (unsigned char ucCold, unsigned char ucWarm)
{
  //Usart1_SendByte ((u8)ucCold);
  //Usart1_SendByte ((u8)ucWarm);
  regulateColdLightPulseWidth (ucCold );
  regulateWarmLightPulseWidth (ucWarm );
  //Usart1_SendByte (ucCold * 100 / 255);
  //Usart1_SendByte (ucWarm * 100 / 255);
}

/******************* (C) COPYRIGHT 2016 Eastfield Lighting *****END OF FILE****/