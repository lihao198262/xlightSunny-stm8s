#include "_global.h"
#include "rf24l01.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "LightPwmDrv.h"

/*
License: MIT

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PC3 -> CE
  PC4 -> CSN
  PC7 -> MISO
  PC6 -> MOSI
  PC5 -> SCK
  PC2 -> IRQ

*/
// Xlight Application Identification
#define XLA_VERSION               0x01
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM
#define XLA_PRODUCT_NAME          "XSunny"                  // Default value. Read from EEPROM

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		71
#define ADDRESS_WIDTH                   5
#define PLOAD_WIDTH                     32

// Unique ID
#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
  #define     UNIQUE_ID_ADDRESS         (0x48CD)
#endif
#if defined(STM8S103) || defined(STM8S003) ||  defined(STM8S903)
  #define     UNIQUE_ID_ADDRESS         (0x4865)
#endif

const UC RF24_BASE_RADIO_ID[ADDRESS_WIDTH] = {0x00,0x54,0x49,0x54,0x44};

// Public variables
Config_t gConfig;
MyMessage_t msg;
uint8_t *pMsg = (uint8_t *)&msg;
bool gIsChanged = FALSE;
uint8_t _uniqueID[UNIQUE_ID_LEN];

// Moudle variables
uint8_t mutex = 0;
uint8_t bMsgReady = 0;
uint8_t rx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t tx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint16_t pwm_Warm = 0;
uint16_t pwm_Cold = 0;

// Delayed operation in function idleProcess()
typedef void (*OnTick_t)(uint16_t);  // Operation callback function typedef
// func bits
uint8_t delay_func = 0x00;
bool delay_up[DELAY_TIMERS];
uint16_t delay_from[DELAY_TIMERS];
uint16_t delay_to[DELAY_TIMERS];
uint16_t delay_step[DELAY_TIMERS];
uint32_t delay_tick[DELAY_TIMERS];
uint32_t delay_timer[DELAY_TIMERS];
OnTick_t delay_handler[DELAY_TIMERS];

void Flash_ReadBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS_OK(Address));
  assert_param(IS_FLASH_ADDRESS_OK(Address+Length));
  
  for( uint16_t i = 0; i < Length; i++ ) {
    Buffer[i] = FLASH_ReadByte(Address+i);
  }
}

void Flash_WriteBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS_OK(Address));
  assert_param(IS_FLASH_ADDRESS_OK(Address+Length));
  
  // Init Flash Read & Write
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  
  uint8_t WriteBuf[FLASH_BLOCK_SIZE];
  uint16_t nBlockNum = (Length - 1) / FLASH_BLOCK_SIZE + 1;
  for( uint16_t block = 0; block < nBlockNum; block++ ) {
    memset(WriteBuf, 0x00, FLASH_BLOCK_SIZE);
    for( uint16_t i = 0; i < FLASH_BLOCK_SIZE; i++ ) {
      WriteBuf[i] = Buffer[block * FLASH_BLOCK_SIZE + i];
    }
    FLASH_ProgramBlock(block, FLASH_MEMTYPE_DATA, FLASH_PROGRAMMODE_STANDARD, WriteBuf);
    FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA);
  }
  
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

// Save config to Flash
void SaveConfig()
{
  if( gIsChanged ) {
    Flash_WriteBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsChanged = FALSE;
  }
}

// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( gConfig.version > XLA_VERSION || gConfig.ring1.BR > 100 || gConfig.ring1.CCT < CT_MIN_VALUE 
       || gConfig.ring1.CCT > CT_MAX_VALUE || gConfig.rfPowerLevel > RF24_PA_MAX ) {
      memset(&gConfig, 0x00, sizeof(gConfig));
      gConfig.version = XLA_VERSION;
      gConfig.nodeID = BASESERVICE_ADDRESS;  // NODEID_MAINDEVICE; BASESERVICE_ADDRESS; NODEID_DUMMY
      gConfig.present = 0;
      gConfig.type = devtypWRing3;
      gConfig.ring1.State = 1;
      gConfig.ring1.BR = DEFAULT_BRIGHTNESS;
      gConfig.ring1.CCT = CT_MIN_VALUE;
      gConfig.ring1.R = 0;
      gConfig.ring1.G = 0;
      gConfig.ring1.B = 0;
      gConfig.ring2 = gConfig.ring1;
      gConfig.ring3 = gConfig.ring1;
      gConfig.rfPowerLevel = RF24_PA_MAX;
      memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
      sprintf(gConfig.Organization, "%s", XLA_ORGANIZATION);
      sprintf(gConfig.ProductName, "%s", XLA_PRODUCT_NAME);
      gIsChanged = TRUE;
      SaveConfig();
    }
}

void UpdateNodeAddress(void) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  tx_addr[0] = (gConfig.nodeID >= BASESERVICE_ADDRESS ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
  RF24L01_setup(tx_addr, rx_addr, RF24_CHANNEL);
}

void Delay(uint32_t _timeout) {
  while(_timeout--);
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( idleProcess() > 0 ) return TRUE;
  }
  return FALSE;
}

void CCT2ColdWarm(uint32_t ucBright, uint32_t ucWarmCold)
{
  if(ucBright>100)
    ucBright = 100;
  if(ucWarmCold>=CT_MAX_VALUE)
    ucWarmCold = CT_MAX_VALUE;
  if(ucWarmCold<=CT_MIN_VALUE)
    ucWarmCold = CT_MIN_VALUE;
  
  ucWarmCold -= CT_MIN_VALUE;
  ucWarmCold*= 10;
  ucWarmCold/=CT_SCOPE;         // 0 - 1000
  
  // Convert brightness with quadratic function 
  ucBright = ucBright * ucBright / 100;
  pwm_Warm = (1000 - ucWarmCold)*ucBright/1000 ;
  pwm_Cold = ucWarmCold*ucBright/1000 ;
}

int main( void ) {
  
  //After reset, the device restarts by default with the HSI clock divided by 8.
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);  // now: HSI=16M prescale = 1; sysclk = 16M
  //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2);  // now: HSI=16M prescale = 16; sysclk = 1M
  
  // Init CCT
  initTim2PWMFunction();
  
  // Go on only if NRF chip is presented
  RF24L01_init();
  //while(NRF24L01_Check());

  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  
  // Bring the lights to the most recent or default light-on status
  DEVST_OnOff = 0;      // Ensure to turn on the light at next step
  SetDeviceOnOff(TRUE); // Always turn light on
  
  WaitMutex(0x1FFFF);   // about 3 sec
  
  // Update RF addresses and Setup RF environment
  //gConfig.nodeID = 0x11; // test
  //gConfig.nodeID = NODEID_MAINDEVICE;   // test
  //gConfig.nodeID = BASESERVICE_ADDRESS;   // test
  //memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH); // test
  UpdateNodeAddress();

  // IRQ
  GPIO_Init(
    GPIOC,
    GPIO_PIN_2,
    GPIO_MODE_IN_FL_IT
  );
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_FALL_ONLY);
  enableInterrupts();

  if( (gConfig.nodeID < NODEID_MIN_DEVCIE || gConfig.nodeID > NODEID_MAX_DEVCIE) && gConfig.nodeID != NODEID_MAINDEVICE ) {
    // Request for NodeID
    build(BASESERVICE_ADDRESS, NODE_TYP_LAMP, C_INTERNAL, I_ID_REQUEST, 1, 0);
    miSetPayloadType(P_ULONG32);
    miSetLength(UNIQUE_ID_LEN);
    memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  } else {
    // Send Presentation Message
    Msg_Presentation();
  }
  
  while(1) {
    mutex = 0;
    RF24L01_set_mode_RX();
    RF24L01_set_mode_TX();
    RF24L01_write_payload(pMsg, PLOAD_WIDTH);
    // Timeout about 10 sec
    if( WaitMutex(0x4FFFF) ) {
      break;
      //if (mutex == 1) break;
      //The transmission failed
      //asm("nop"); //Place a breakpoint here to see memory
      // Delay for a while to wait controller started
      //Delay(0x8FFFF);   // about 20 sec
    }
    // Timeout, then repeat init-step
    UpdateNodeAddress();
  }

  while (1) {
    // Receive and process message
    mutex = 0;
    bMsgReady = 0;
    RF24L01_set_mode_RX();
    while(!idleProcess());
    if (mutex == 1) {
      RF24L01_read_payload(pMsg, PLOAD_WIDTH);
      bMsgReady = ParseProtocol();
    }
    else {
      //Something happened
      WaitMutex(0xFFF);
      continue;
    }
    
    // Save Config if Changed
    SaveConfig();
    
    // Send message
    while( bMsgReady == 1 ) {
      mutex = 0;
      bMsgReady = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(pMsg, PLOAD_WIDTH);

      // may have more messages
      WaitMutex(0x1FFFF);
      if (mutex != 1) {
        //The transmission failed, Notes: mutex == 2 doesn't mean failed
        //It happens when rx address defers from tx address
        //asm("nop"); //Place a breakpoint here to see memory
      }
    }
  }
}

void ChangeDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct) {
  CCT2ColdWarm(_sw ? _br : 0, _cct);
  driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
}

void ChangeDeviceBR(uint16_t _br) {
  ChangeDeviceStatus(TRUE, (uint8_t)_br, DEVST_WarmCold);
}

void ChangeDeviceCCT(uint16_t _cct) {
  ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, _cct);
}

void DelaySendMsg(uint16_t _msg) {
  switch( _msg ) {
  case 1:
    // send current on/off status
    Msg_DevOnOff(NODEID_GATEWAY, NODEID_MIN_REMOTE);
    bMsgReady = 1;
    break;
    
  case 2:
    // send current brigntness status
    Msg_DevBrightness(NODEID_GATEWAY, NODEID_MIN_REMOTE);
    bMsgReady = 1;
    break;
  }
  
  if( bMsgReady ) Delay(0x1FF);
}

bool SetDeviceOnOff(bool _sw) {
  if( _sw != DEVST_OnOff ) {
    uint8_t _Brightness = (DEVST_Bright >= BR_MIN_VALUE ? DEVST_Bright : DEFAULT_BRIGHTNESS);

    DEVST_OnOff = _sw;
    if( _Brightness != DEVST_Bright ) {
      DEVST_Bright = _Brightness;

      // Inform the controller in order to keep consistency
      /*
      delay_from[DELAY_TIM_MSG] = 2;    // Must be the MsgID
      delay_to[DELAY_TIM_MSG] = 2;      // Must be the MsgID
      delay_up[DELAY_TIM_MSG] = TRUE;
      delay_step[DELAY_TIM_MSG] = 1;
      delay_timer[DELAY_TIM_MSG] = 0xFF;
      delay_tick[DELAY_TIM_MSG] = 0;      // execute next step right away
      delay_handler[DELAY_TIM_MSG] = DelaySendMsg;
      BF_SET(delay_func, 1, DELAY_TIM_MSG, 1);
      */
    }
    
#ifdef GRADUAL_ONOFF

    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_ONOFF] = (_sw ? BR_MIN_VALUE : _Brightness);
    delay_to[DELAY_TIM_ONOFF] = (_sw ? _Brightness : BR_MIN_VALUE);
    delay_up[DELAY_TIM_ONOFF] = (delay_from[DELAY_TIM_ONOFF] < delay_to[DELAY_TIM_ONOFF]);
    delay_step[DELAY_TIM_ONOFF] = BRIGHTNESS_STEP;

    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_ONOFF] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_ONOFF] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_ONOFF] = ChangeDeviceBR;
    BF_SET(delay_func, 1, DELAY_TIM_ONOFF, 1); // Enable OnOff operation
    
#else

    // To the final status
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold);

#endif    

    gIsChanged = TRUE;
    return TRUE;
  }
  
  return FALSE;
}

bool SetDeviceBrightness(uint8_t _br) {
  if( _br != DEVST_Bright ) {
#ifdef GRADUAL_ONOFF    
    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_BR] = DEVST_Bright;
    delay_to[DELAY_TIM_BR] = _br;
    delay_up[DELAY_TIM_BR] = (delay_from[DELAY_TIM_BR] < delay_to[DELAY_TIM_BR]);
    delay_step[DELAY_TIM_BR] = BRIGHTNESS_STEP;
#endif
    
    bool newSW = (_br >= BR_MIN_VALUE);
    DEVST_Bright = _br;
    if( DEVST_OnOff != newSW ) {
      DEVST_OnOff = newSW;
      // Inform the controller in order to keep consistency
      /*
      delay_from[DELAY_TIM_MSG] = 1;    // Must be the MsgID
      delay_to[DELAY_TIM_MSG] = 1;      // Must be the MsgID
      delay_up[DELAY_TIM_MSG] = TRUE;
      delay_step[DELAY_TIM_MSG] = 1;
      delay_timer[DELAY_TIM_MSG] = 0xFF;
      delay_tick[DELAY_TIM_MSG] = 0;      // execute next step right away
      delay_handler[DELAY_TIM_MSG] = DelaySendMsg;
      BF_SET(delay_func, 1, DELAY_TIM_MSG, 1);
      */
    }
    
#ifdef GRADUAL_ONOFF
    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_BR] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_BR] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_BR] = ChangeDeviceBR;
    BF_SET(delay_func, 1, DELAY_TIM_BR, 1); // Enable BR Dimmer operation
#else    
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold);
#endif

    gIsChanged = TRUE;
    return TRUE;
  }
  
  return FALSE;
}

bool SetDeviceCCT(uint16_t _cct) {
  if( _cct != DEVST_WarmCold ) {
#ifdef GRADUAL_CCT    
    // Smoothly change CCT - set parameters
    delay_from[DELAY_TIM_CCT] = DEVST_WarmCold;
    delay_to[DELAY_TIM_CCT] = _cct;
    delay_up[DELAY_TIM_CCT] = (delay_from[DELAY_TIM_CCT] < delay_to[DELAY_TIM_CCT]);
    delay_step[DELAY_TIM_CCT] = CCT_STEP;
#endif
    
    DEVST_WarmCold = _cct;
    
#ifdef GRADUAL_CCT
    // Smoothly change CCT - set timer
    delay_timer[DELAY_TIM_CCT] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_CCT] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_CCT] = ChangeDeviceCCT;
    BF_SET(delay_func, 1, DELAY_TIM_CCT, 1); // Enable CCT Dimmer operation
#else    
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold);
#endif
    
    gIsChanged = TRUE;
    return TRUE;
  }
  
  return FALSE;
}

bool isTimerCompleted(uint8_t _tmr) {
  bool bFinished;
  
  if( delay_up[_tmr] ) {
    // Up
    delay_from[_tmr] += delay_step[_tmr];
    bFinished = ( delay_from[_tmr] >= delay_to[_tmr] );
  } else {
    // Down
    delay_from[_tmr] -= delay_step[_tmr];
    bFinished = ( delay_from[_tmr] <= delay_to[_tmr] );
  }
  
  // Execute operation
  if( delay_handler[_tmr] ) {
    if( bFinished ) {
      // Completed - to the final status
      (*delay_handler[_tmr])(delay_to[_tmr]);
    } else {
      // Progress
      (*delay_handler[_tmr])(delay_from[_tmr]);
    }
  }
  
  return bFinished;
}

// Execute delayed operations
uint8_t idleProcess() {
  for( uint8_t _tmr = 0; _tmr < DELAY_TIMERS; _tmr++ ) {
    if( BF_GET(delay_func, _tmr, 1) ) {
      // Timer is enabled
      if( delay_tick[_tmr] == 0 ) {
        // Timer reached, reset it
        delay_tick[_tmr] = delay_timer[_tmr];
        // Move a step and execute operation
        if( isTimerCompleted(_tmr) ) {
          // Stop timer - disable further operation
          BF_SET(delay_func, 0, _tmr, 1);
        }
      } else {
        // Timer not reached, tick it
        delay_tick[_tmr]--;
      }
    }
  }
  
  return mutex; 
}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    mutex = 1;
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info;
    return;
  }

   RF24L01_clear_interrupts();
}
