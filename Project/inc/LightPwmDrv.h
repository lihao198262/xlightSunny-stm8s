/*;*******************************************************************************
; FILENAME		: LightPwmDrv.h
; AUTHOR		: Feng Liu
; PURPOSE		: PWM-based Lighting Driver
; REVISION		: 1.0
; SYSTEM CLOCK	        : 16MHz
; MCU type              : STM8L051F3            
; Date Time             : 2016-04-18
; Copy right :          : Eastfield Lighting Co., Ltd.
;*******************************************************************************/

#ifndef __LIGHT_PWM_DRIVER_
#define __LIGHT_PWM_DRIVER_

/* Includes ------------------------------------------------------------------*/
#include "_global.h"

/* Exported variables ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#define MODE_CC1_EXTERNAL_PWM_DUTY      
void callbackTim1EventCC (void);
void callbackTim3EventCC (void);

void initExternalPwmCaptureFunction (void);
void initTim2PWMFunction (void);

void driveColdWarmLightPwm (unsigned char ucCold, unsigned char ucWarm);

void LightRGBWBRCtrl(uint8_t RValue, uint8_t GValue, uint8_t BValue, uint8_t WValue, uint8_t BRPercent);
void LightRGBBRCtrl(uint8_t RValue, uint8_t GValue, uint8_t BValue, uint8_t BRPercent);

#if defined(XRAINBOW) || defined(XMIRAGE)
void ConvertCCT2RGBW(uint16_t CCTValue, uint8_t *RValue, uint8_t *GValue, uint8_t *BValue, uint8_t *WValue);
#endif

#endif
/******************* (C) COPYRIGHT 2016 Eastfield Lighting *****END OF FILE****/