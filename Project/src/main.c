#include "_global.h"
#include "delay.h"
#include "rf24l01.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "ProtocolParser.h"
#include "LightPwmDrv.h"
#include "Uart2Dev.h"

#ifdef EN_SENSOR_ALS
#include "ADC1Dev.h"
#include "sen_als.h"
#endif

#ifdef EN_SENSOR_PIR
#include "sen_pir.h"
#endif

/*
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

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

// Simple Direct Test
// Uncomment this line to work in Simple Direct Test Mode
//#define ENABLE_SDTM

// Xlight Application Identification
#define XLA_VERSION               0x03
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM

// Choose Product Name & Type
/// Sunny
#if defined(XSUNNY)
#define XLA_PRODUCT_NAME          "XSunny"
#define XLA_PRODUCT_Type          devtypWRing3
#endif
/// Rainbow
#if defined(XRAINBOW)
#define XLA_PRODUCT_NAME          "XRainbow"
#define XLA_PRODUCT_Type          devtypCRing3
#endif
/// Mirage
#if defined(XMIRAGE)
#define XLA_PRODUCT_NAME          "XMirage"
#define XLA_PRODUCT_Type          devtypMRing3
#endif

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		71
#define ADDRESS_WIDTH                   5
#define PLOAD_WIDTH                     32

// Window Watchdog
// Uncomment this line if in debug mode
#define DEBUG_NO_WWDG
#define WWDG_COUNTER                    0x7f
#define WWDG_WINDOW                     0x77

// System Startup Status
#define SYS_INIT                        0
#define SYS_RESET                       1
#define SYS_WAIT_NODEID                 2
#define SYS_WAIT_PRESENTED              3
#define SYS_RUNNING                     5

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
uint8_t mStatus = SYS_INIT;
bool mGotNodeID = FALSE;
bool mGotPresented = FALSE;
uint8_t mutex = 0;
uint8_t rx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t tx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint16_t pwm_Warm = 0;
uint16_t pwm_Cold = 0;

// USART
uint8_t USART_ReceiveDataBuf[32];
uint8_t Buff_Cnt;
uint8_t USART_FLAG;
uint8_t RX_TX_BUFF = 0;

// Delayed operation in function idleProcess()
typedef void (*OnTick_t)(uint32_t, uint8_t);  // Operation callback function typedef
// func bits
uint8_t delay_func = 0x00;
bool delay_up[DELAY_TIMERS];
uint32_t delay_from[DELAY_TIMERS];
uint32_t delay_to[DELAY_TIMERS];
uint32_t delay_step[DELAY_TIMERS];
uint32_t delay_tick[DELAY_TIMERS];
uint32_t delay_timer[DELAY_TIMERS];
OnTick_t delay_handler[DELAY_TIMERS];
uint8_t delay_tag[DELAY_TIMERS];

// Initialize Window Watchdog
void wwdg_init() {
#ifndef DEBUG_NO_WWDG  
  WWDG_Init(WWDG_COUNTER, WWDG_WINDOW);
#endif  
}

// Feed the Window Watchdog
void feed_wwdg(void) {
#ifndef DEBUG_NO_WWDG    
  uint8_t cntValue = WWDG_GetCounter() & WWDG_COUNTER;
  if( cntValue < WWDG_WINDOW ) {
    WWDG_SetCounter(WWDG_COUNTER);
  }
#endif  
}

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

bool isIdentityEmpty(const UC *pId, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId[i] > 0) return FALSE; }
  return TRUE;
}

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId1[i] != pId2[i]) return FALSE; }
  return TRUE;
}

bool isNodeIdRequired()
{
  return( (IS_NOT_DEVICE_NODEID(gConfig.nodeID) && !IS_GROUP_NODEID(gConfig.nodeID)) || 
         isIdentityEmpty(gConfig.NetworkID, ADDRESS_WIDTH) || isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) );
}

// Save config to Flash
void SaveConfig()
{
#ifndef ENABLE_SDTM  
  if( gIsChanged ) {
    Flash_WriteBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsChanged = FALSE;
  }
#endif  
}

// Initialize Node Address and look forward to being assigned with a valid NodeID by the SmartController
void InitNodeAddress() {
  // Whether has preset node id
  if( IS_NOT_DEVICE_NODEID(gConfig.nodeID) && !IS_GROUP_NODEID(gConfig.nodeID) ) {
    gConfig.nodeID = BASESERVICE_ADDRESS; // NODEID_MAINDEVICE; BASESERVICE_ADDRESS; NODEID_DUMMY
  }
  memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
}

// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( gConfig.version > XLA_VERSION || DEVST_Bright > 100 || gConfig.rfPowerLevel > RF24_PA_MAX 
       || gConfig.type != XLA_PRODUCT_Type 
       || strcmp(gConfig.Organization, XLA_ORGANIZATION) != 0  ) {
      memset(&gConfig, 0x00, sizeof(gConfig));
      gConfig.version = XLA_VERSION;
      InitNodeAddress();
      gConfig.present = 0;
      gConfig.type = XLA_PRODUCT_Type;
      gConfig.ring[0].State = 1;
      gConfig.ring[0].BR = DEFAULT_BRIGHTNESS;
#if defined(XSUNNY)      
      gConfig.ring[0].CCT = CT_MIN_VALUE;
#else
      gConfig.ring[0].CCT = 0;
#endif      
      gConfig.ring[0].R = 0;
      gConfig.ring[0].G = 0;
      gConfig.ring[0].B = 0;
      gConfig.ring[1] = gConfig.ring[0];
      gConfig.ring[2] = gConfig.ring[0];
      gConfig.rfPowerLevel = RF24_PA_MAX;
      gConfig.hasSiblingMCU = 0;
      sprintf(gConfig.Organization, "%s", XLA_ORGANIZATION);
      sprintf(gConfig.ProductName, "%s", XLA_PRODUCT_NAME);
      
      gConfig.senMap = 0;
#ifdef EN_SENSOR_ALS
      gConfig.senMap |= sensorALS;
#endif
#ifdef EN_SENSOR_PIR
      gConfig.senMap |= sensorPIR;
#endif
#ifdef EN_SENSOR_DHT
      gConfig.senMap |= sensorDHT;
#endif      
      gConfig.funcMap = 0;
      gConfig.alsLevel[0] = 70;
      gConfig.alsLevel[1] = 90;

      gIsChanged = TRUE;
      SaveConfig();
    }
}

void UpdateNodeAddress(void) {
#ifdef ENABLE_SDTM
  RF24L01_setup(tx_addr, rx_addr, RF24_CHANNEL, 0);     // Without openning the boardcast pipe
#else  
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  tx_addr[0] = (isNodeIdRequired() ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
  RF24L01_setup(tx_addr, rx_addr, RF24_CHANNEL, BROADCAST_ADDRESS);     // With openning the boardcast pipe
#endif  
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( idleProcess() > 0 ) return TRUE;
  }
  return FALSE;
}

uint8_t GetSteps(uint32_t _from, uint32_t _to)
{
  uint8_t _step = BRIGHTNESS_STEP;
  uint32_t _gap;
  if( _from > _to ) {
    _gap = _from - _to;
  } else {
    _gap = _to - _from;
  }
  // Max 40 times
  if( _step * MAX_STEP_TIMES < _gap ) {
    _step = _gap / MAX_STEP_TIMES + 1;
  }
  return _step;
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
  // Func 1
  // y = (100 - b) * x * sqrt(x) / 1000 + b
  // , where b = LIGHT_PWM_THRESHOLD
  // Func 2
  // y = (100 - b) * x * x / 10000 + b
  // , where b = LIGHT_PWM_THRESHOLD
  if( ucBright > 0 ) {
    //float rootBright = sqrt(ucBright);
    //ucBright = (uint32_t)((100 - LIGHT_PWM_THRESHOLD) * ucBright * rootBright / 1000 + LIGHT_PWM_THRESHOLD + 0.5);
    ucBright = (100 - LIGHT_PWM_THRESHOLD) * ucBright * ucBright / 10000 + LIGHT_PWM_THRESHOLD + 0.5;
  }
  
  pwm_Warm = (1000 - ucWarmCold)*ucBright/1000 ;
  pwm_Cold = ucWarmCold*ucBright/1000 ;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // delay to avoid conflict
    if( bDelaySend ) {
      delay_ms(gConfig.nodeID % 25 * 10);
      bDelaySend = FALSE;
    }

    mutex = 0;
    RF24L01_set_mode_TX();
    RF24L01_write_payload(pMsg, PLOAD_WIDTH);

    WaitMutex(0x1FFFF);
    if (mutex != 1) {
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
  }

  return(mutex > 0);
}

void GotNodeID() {
  mGotNodeID = TRUE;
  UpdateNodeAddress();
  SaveConfig();
}

void GotPresented() {
  mGotPresented = TRUE;
}

bool SayHelloToDevice(bool infinate) {
  uint8_t _count = 0;
  uint8_t _presentCnt = 0;
  bool _doNow = FALSE;

  // Update RF addresses and Setup RF environment
  UpdateNodeAddress();

  while(1) {
    if( _count++ == 0 ) {
      
      if( isNodeIdRequired() ) {
        mStatus = SYS_WAIT_NODEID;
      } else {
        mStatus = SYS_WAIT_PRESENTED;
      }
      
      _doNow = FALSE;
      if( mStatus == SYS_WAIT_NODEID ) {
        // Request for NodeID
        Msg_RequestNodeID();
        mGotNodeID = FALSE;
      } else {
        // Send Presentation Message
        Msg_Presentation();
        mGotPresented = FALSE;
        _presentCnt++;
      }
      
      if( !SendMyMessage() ) {
        if( !infinate ) return FALSE;
      } else {
        // Wait response
        uint16_t tick = 0xAFFF;
        while(tick--) {
          // Feed the Watchdog
          feed_wwdg();
          if( mStatus == SYS_WAIT_NODEID && mGotNodeID ) {
            mStatus = SYS_WAIT_PRESENTED;
            _presentCnt = 0;
            _doNow = TRUE;
            break;
          }
          if( mStatus == SYS_WAIT_PRESENTED && mGotPresented ) {
            mStatus = SYS_RUNNING;
            return TRUE;
          }
        }
      }
    }

    // Can't presented for a few times, then try request NodeID again
    // Either because SmartController is off, or changed
    if(  mStatus == SYS_WAIT_PRESENTED && _presentCnt >= 5 ) {
      _presentCnt = 0;
      // Reset RF Address
      InitNodeAddress();
      UpdateNodeAddress();
      mStatus = SYS_WAIT_NODEID;
      _doNow = TRUE;
    }
    
    if( _doNow ) {
      // Send Message Immediately
      _count = 0;
      continue;
    }
    
    // Feed the Watchdog
    feed_wwdg();
    
    // Failed or Timeout, then repeat init-step
    delay_ms(400);
    _count %= 20;  // Every 10 seconds
  }
  
  return TRUE;
}

int main( void ) {
#ifdef EN_SENSOR_ALS
   static uint8_t pre_als_value = 0;
   uint8_t als_value;
   uint16_t als_tick = 0;
   uint8_t lv_Brightness;
#endif

#ifdef EN_SENSOR_PIR
   static bool pre_pir_st = FALSE;
   bool pir_st;
   uint16_t pir_tick = 0;
#endif
   
   
  //After reset, the device restarts by default with the HSI clock divided by 8.
  //CLK_DeInit();
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);  // now: HSI=16M prescale = 1; sysclk = 16M
  //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2);  // now: HSI=16M prescale = 2; sysclk = 8M
  //CLK_HSECmd(ENABLE);
  // Init PWM Timers
  initTim2PWMFunction();

  // Init sensors
#ifdef EN_SENSOR_ALS  
  als_init();
#endif
#ifdef EN_SENSOR_PIR
  pir_init();
#endif  
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();

  // Init ADC
#ifdef EN_SENSOR_ALS  
  ADC1_Config();
#endif  
  
  // Init serial ports
  uart2_config();
  
  while(1) {
    // Go on only if NRF chip is presented
    RF24L01_init();
    while(!NRF24L01_Check());

    // Try to communicate with sibling MCUs (STM8S003F), 
    /// if got response, which means the device supports indiviual ring control.
    /// Also need to enable RING_INDIVIDUAL_COLOR condition for indiviual ring control.
    // ToDo:
    // gConfig.hasSiblingMCU = PingSiblingMCU();
    
    // Bring the lights to the most recent or default light-on status
    if( mStatus == SYS_INIT ) {
      DEVST_OnOff = 0;      // Ensure to turn on the light at next step
      SetDeviceOnOff(TRUE, RING_ID_ALL); // Always turn light on
      //delay_ms(1500);   // about 1.5 sec
      WaitMutex(0xFFFF); // use this line to bring the lights to target brightness

      // Init Watchdog
      wwdg_init();
    }
  
    // IRQ
    NRF2401_EnableIRQ();
  
#ifdef ENABLE_SDTM
    gConfig.nodeID = 0x11;
    UpdateNodeAddress();
    Msg_DevStatus(0x11, 0x11, RING_ID_ALL);
    SendMyMessage();
    mStatus = SYS_RUNNING;
#else  
    // Must establish connection firstly
    SayHelloToDevice(TRUE);
#endif
    
  
    while (mStatus == SYS_RUNNING) {
      // Feed the Watchdog
      feed_wwdg();
      
      // Read sensors
#ifdef EN_SENSOR_PIR
      /// Read PIR
      if( gConfig.senMap & sensorPIR ) {
        if( !bMsgReady && !pir_tick ) {
          // Reset read timer
          pir_tick = SEN_READ_PIR;
          pir_st = pir_read();
          if( pre_pir_st != pir_st ) {
            // Send detection message
            pre_pir_st = pir_st;
            Msg_SenPIR(pre_pir_st);
            // Action
            if( gConfig.funcMap & controlPIR ) {
              SendMyMessage();
              SetDeviceOnOff(pir_st, RING_ID_ALL);
              Msg_DevBrightness(NODEID_GATEWAY, NODEID_GATEWAY);
            }
          }
        } else if( pir_tick > 0 ) {
          pir_tick--;
        }
      }
#endif

#ifdef EN_SENSOR_ALS
      /// Read ALS
      if( gConfig.senMap & sensorALS ) {
        if( !bMsgReady && !als_tick ) {
          // Reset read timer
          als_tick = SEN_READ_ALS;
          als_value = als_read();
          if( pre_als_value != als_value ) {
            // Send brightness message
            pre_als_value = als_value;
            Msg_SenALS(pre_als_value);

            // Action
            if( gConfig.funcMap & controlALS ) {
              if( DEVST_OnOff ) {
                if( als_value < gConfig.alsLevel[0] || als_value > gConfig.alsLevel[1] ) {
                  lv_Brightness = (gConfig.alsLevel[0] + gConfig.alsLevel[1]) / 2;
                  if( lv_Brightness > 0 && lv_Brightness <= 100 ) {
                    SendMyMessage();
                    SetDeviceBrightness(lv_Brightness, RING_ID_ALL);
                    Msg_DevBrightness(NODEID_GATEWAY, NODEID_GATEWAY);
                  }
                }
              }
            }
          }
        } else if( als_tick > 0 ) {
          als_tick--;
        }
      }
#endif
      
      // Send message if ready
      SendMyMessage();
      
      // Save Config if Changed
      SaveConfig();
      
      // Idle process
      idleProcess();
      
      // ToDo: Check heartbeats
      // mStatus = SYS_REST, if timeout or received a value 3 times consecutively
    }
  }
}

#if defined(XSUNNY)
// Immediately change brightness and cct
void ChangeDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct, uint8_t _ring) {
  CCT2ColdWarm(_sw ? _br : 0, _cct);
  
  if( gConfig.hasSiblingMCU ) {
    // ring individual control
    /*
    uint8_t sendBuf[8];
    sendBuf[0] = UART_CMD_CCT;
    sendBuf[1] = pwm_Cold;
    sendBuf[2] = pwm_Warm;
    sendBuf[3] = '\n';
    sendBuf[4] = '\0';
    */
    if( _ring == RING_ID_ALL ) { 
      // change ring one by one
      driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
      //Uart2SendString(sendBuf);
    } else if( _ring == RING_ID_1 ) {
      // change one specific ring
      driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
    } else if( _ring == RING_ID_2 ) {
      //Uart2SendString(sendBuf);
    } else if( _ring == RING_ID_3 ) {
    }
  } else {
    // Control all rings together
    driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
  }
}
#endif

#if defined(XRAINBOW) || defined(XMIRAGE)
// Immediately change brightness and rgbw
void ChangeDeviceStatus(bool _sw, uint8_t _br, uint32_t _w, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _ring) {
  if(!_sw) {
    _br = 0;
  } else if(_br > 100) {
    _br = 100;
  }
  
  if( gConfig.hasSiblingMCU ) {
    // ring individual control
    /*
    uint8_t sendBuf[8];
    sendBuf[0] = UART_CMD_RGBW;
    sendBuf[1] = _br;
    sendBuf[2] = _r;
    sendBuf[3] = _g;
    sendBuf[4] = _b;
    sendBuf[5] = _w;
    sendBuf[6] = '\n';
    sendBuf[7] = '\0';
    */
    if( _ring == RING_ID_ALL ) { 
      // change ring one by one
      LightRGBWBRCtrl(_r, _g, _b, _w, _br);
      //Uart2SendString(sendBuf);
    } else if( _ring == RING_ID_1 ) {
      // change one specific ring
      LightRGBWBRCtrl(_r, _g, _b, _w, _br);
    } else if( _ring == RING_ID_2 ) {
      //Uart2SendString(sendBuf);
    } else if( _ring == RING_ID_3 ) {
    }
  } else {
    // Control all rings together
    LightRGBWBRCtrl(_r, _g, _b, _w, _br);
  }
}
#endif

// Immediately change brightness
void ChangeDeviceBR(uint32_t _br, uint8_t _ring) {
#ifdef RING_INDIVIDUAL_COLOR
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
#if defined(XSUNNY)
  ChangeDeviceStatus(TRUE, (uint8_t)_br, RINGST_WarmCold(r_index), _ring);
#endif
#if defined(XRAINBOW) || defined(XMIRAGE)
  ChangeDeviceStatus(TRUE, (uint8_t)_br, RINGST_W(r_index), RINGST_R(r_index), RINGST_G(r_index), RINGST_B(r_index), _ring);
#endif
#else
#if defined(XSUNNY)
  ChangeDeviceStatus(TRUE, (uint8_t)_br, DEVST_WarmCold, _ring);
#endif
#if defined(XRAINBOW) || defined(XMIRAGE)
  ChangeDeviceStatus(TRUE, (uint8_t)_br, DEVST_W, DEVST_R, DEVST_G, DEVST_B, _ring);
#endif
#endif
}

// Immediately change cct
void ChangeDeviceCCT(uint32_t _cct, uint8_t _ring) {
#ifdef RING_INDIVIDUAL_COLOR
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
#if defined(XSUNNY)
  ChangeDeviceStatus(RINGST_OnOff(r_index), RINGST_Bright(r_index), (uint16_t)_cct, _ring);
#endif
#if defined(XRAINBOW) || defined(XMIRAGE)
  // ToDo: calcualte RGB from Warm White
  ChangeDeviceStatus(RINGST_OnOff(r_index), RINGST_Bright(r_index), (uint8_t)_cct, RINGST_R(r_index), RINGST_G(r_index), RINGST_B(r_index), _ring);
#endif
#else
#if defined(XSUNNY)
  ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, (uint16_t)_cct, _ring);
#endif
#if defined(XRAINBOW) || defined(XMIRAGE)
  // ToDo: calcualte RGB from Warm White
  uint8_t _w, _r, _g, _b;
  ConvertCCT2RGBW(_cct, &_r, &_g, &_b, &_w);
  ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, _w, _r, _g, _b, _ring);
#endif
#endif
}

// Immediately change wrgb
void ChangeDeviceWRGB(uint32_t _wrgb, uint8_t _ring) {
  uint8_t _w, _r, _g, _b;
  _b = (_wrgb & 0xff);
  _wrgb >>= 8;
  _g = (_wrgb & 0xff);
  _wrgb >>= 8;
  _r = (_wrgb & 0xff);
  _wrgb >>= 8;
  _w = (_wrgb & 0xff);
  
#ifdef RING_INDIVIDUAL_COLOR
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
#if defined(XRAINBOW) || defined(XMIRAGE)
  ChangeDeviceStatus(TRUE, RINGST_Bright(r_index), _w, _r, _g, _b, _ring);
#endif
#else
#if defined(XRAINBOW) || defined(XMIRAGE)
  ChangeDeviceStatus(TRUE, DEVST_Bright, _w, _r, _g, _b, _ring);
#endif
#endif
}

void DelaySendMsg(uint16_t _msg, uint8_t _ring) {
  switch( _msg ) {
  case 1:
    // send current on/off status
    Msg_DevOnOff(NODEID_GATEWAY, NODEID_MIN_REMOTE);
    break;
    
  case 2:
    // send current brigntness status
    Msg_DevBrightness(NODEID_GATEWAY, NODEID_MIN_REMOTE);
    break;
  }
  
  if( bMsgReady ) delay_10us(20);
}

// Gradually turn on or off
bool SetDeviceOnOff(bool _sw, uint8_t _ring) {
  
#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  if( _sw != RINGST_OnOff(r_index) ) {
    uint8_t _Brightness = (RINGST_Bright(r_index) >= BR_MIN_VALUE ? RINGST_Bright(r_index) : DEFAULT_BRIGHTNESS);

    RINGST_OnOff(r_index) = _sw;
    if( _Brightness != RINGST_Bright(r_index) ) {
      RINGST_Bright(r_index) = _Brightness;
    }
    
#ifdef GRADUAL_ONOFF

    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_ONOFF] = (_sw ? BR_MIN_VALUE : _Brightness);
    delay_to[DELAY_TIM_ONOFF] = (_sw ? _Brightness : 0);
    delay_up[DELAY_TIM_ONOFF] = (delay_from[DELAY_TIM_ONOFF] < delay_to[DELAY_TIM_ONOFF]);
    delay_step[DELAY_TIM_ONOFF] = GetSteps(delay_from[DELAY_TIM_ONOFF], delay_to[DELAY_TIM_ONOFF]);

    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_ONOFF] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_ONOFF] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_ONOFF] = ChangeDeviceBR;
    delay_tag[DELAY_TIM_ONOFF] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_ONOFF, 1); // Enable OnOff operation
    
#else

    // To the final status
    ChangeDeviceStatus(RINGST_OnOff(r_index), RINGST_Bright(r_index), RINGST_WarmCold(r_index), _ring);

#endif    

    gIsChanged = TRUE;
    return TRUE;
  }
  
#else
  
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
      delay_tag[DELAY_TIM_MSG] = _ring;
      BF_SET(delay_func, 1, DELAY_TIM_MSG, 1);
      */
    }
    
#ifdef GRADUAL_ONOFF

    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_ONOFF] = (_sw ? BR_MIN_VALUE : _Brightness);
    delay_to[DELAY_TIM_ONOFF] = (_sw ? _Brightness : 0);
    delay_up[DELAY_TIM_ONOFF] = (delay_from[DELAY_TIM_ONOFF] < delay_to[DELAY_TIM_ONOFF]);
    // Get Step
    delay_step[DELAY_TIM_ONOFF] = GetSteps(delay_from[DELAY_TIM_ONOFF], delay_to[DELAY_TIM_ONOFF]);

    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_ONOFF] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_ONOFF] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_ONOFF] = ChangeDeviceBR;
    delay_tag[DELAY_TIM_ONOFF] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_ONOFF, 1); // Enable OnOff operation
    
#else

    // To the final status
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold, _ring);

#endif    

    gIsChanged = TRUE;
    return TRUE;
  }

#endif
  
  return FALSE;
}

// Gradually change brightness
bool SetDeviceBrightness(uint8_t _br, uint8_t _ring) {

#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  if( _br != RINGST_Bright(r_index) ) {
#ifdef GRADUAL_ONOFF    
    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_BR] = RINGST_Bright(r_index);
    delay_to[DELAY_TIM_BR] = _br;
    delay_up[DELAY_TIM_BR] = (delay_from[DELAY_TIM_BR] < delay_to[DELAY_TIM_BR]);
    delay_step[DELAY_TIM_BR] = GetSteps(delay_from[DELAY_TIM_BR], delay_to[DELAY_TIM_BR]);
#endif
    
    bool newSW = (_br >= BR_MIN_VALUE);
    RINGST_Bright(r_index) = _br;
    if( RINGST_OnOff(r_index) != newSW ) {
      RINGST_OnOff(r_index) = newSW;
    }
    
#ifdef GRADUAL_ONOFF
    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_BR] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_BR] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_BR] = ChangeDeviceBR;
    delay_tag[DELAY_TIM_BR] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_BR, 1); // Enable BR Dimmer operation
#else    
    ChangeDeviceStatus(RINGST_OnOff(r_index), RINGST_Bright(r_index), RINGST_WarmCold(r_index), _ring);
#endif

    gIsChanged = TRUE;
    return TRUE;
  }
  
#else
  
  if( _br != DEVST_Bright ) {
#ifdef GRADUAL_ONOFF    
    // Smoothly change brightness - set parameters
    delay_from[DELAY_TIM_BR] = DEVST_Bright;
    delay_to[DELAY_TIM_BR] = _br;
    delay_up[DELAY_TIM_BR] = (delay_from[DELAY_TIM_BR] < delay_to[DELAY_TIM_BR]);
    delay_step[DELAY_TIM_BR] = GetSteps(delay_from[DELAY_TIM_BR], delay_to[DELAY_TIM_BR]);
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
      delay_tag[DELAY_TIM_MSG] = _ring;
      BF_SET(delay_func, 1, DELAY_TIM_MSG, 1);
      */
    }
    
#ifdef GRADUAL_ONOFF
    // Smoothly change brightness - set timer
    delay_timer[DELAY_TIM_BR] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_BR] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_BR] = ChangeDeviceBR;
    delay_tag[DELAY_TIM_BR] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_BR, 1); // Enable BR Dimmer operation
#else    
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold, _ring);
#endif

    gIsChanged = TRUE;
    return TRUE;
  }
  
#endif
  
  return FALSE;
}

// Gradually change cct
bool SetDeviceCCT(uint16_t _cct, uint8_t _ring) {
  
  if( _cct > CT_MAX_VALUE ) {
    _cct = CT_MAX_VALUE;
  } else if( _cct < CT_MIN_VALUE ) {
    _cct = CT_MIN_VALUE;
  }
  
#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  if( _cct != RINGST_WarmCold(r_index) ) {
#ifdef GRADUAL_CCT    
    // Smoothly change CCT - set parameters
    delay_from[DELAY_TIM_CCT] = RINGST_WarmCold(r_index);
    delay_to[DELAY_TIM_CCT] = _cct;
    delay_up[DELAY_TIM_CCT] = (delay_from[DELAY_TIM_CCT] < delay_to[DELAY_TIM_CCT]);
    delay_step[DELAY_TIM_CCT] = CCT_STEP;
#endif
    
    RINGST_WarmCold(r_index) = _cct;
    
#ifdef GRADUAL_CCT
    // Smoothly change CCT - set timer
    delay_timer[DELAY_TIM_CCT] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_CCT] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_CCT] = ChangeDeviceCCT;
    delay_tag[DELAY_TIM_CCT] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_CCT, 1); // Enable CCT Dimmer operation
#else    
    ChangeDeviceStatus(RINGST_OnOff(r_index), RINGST_Bright(r_index), RINGST_WarmCold(r_index), _ring);
#endif
    
    gIsChanged = TRUE;
    return TRUE;
  }
  
#else
  
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
    delay_tag[DELAY_TIM_CCT] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_CCT, 1); // Enable CCT Dimmer operation
#else
#if defined(XSUNNY)
    ChangeDeviceStatus(DEVST_OnOff, DEVST_Bright, DEVST_WarmCold, _ring);
#endif    
#if defined(XRAINBOW) || defined(XMIRAGE)
    ChangeDeviceCCT(DEVST_WarmCold, _ring);
#endif    
#endif
    
    gIsChanged = TRUE;
    return TRUE;
  }
  
#endif
  
  return FALSE;
}

// Gradually change WRGB
bool SetDeviceWRGB(uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _ring) {
  
#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  if( _w != RINGST_W(r_index) || _r != RINGST_R(r_index) || _g != RINGST_G(r_index) || _b != RINGST_B(r_index) ) {
#ifdef GRADUAL_RGB    
    // Smoothly change RGBW - set parameters
    delay_from[DELAY_TIM_RGB] = RINGST_W(r_index);
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += RINGST_R(r_index);
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += RINGST_G(r_index);
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += RINGST_B(r_index);
    delay_to[DELAY_TIM_RGB] = _w;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _r;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _g;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _b;    
    delay_up[DELAY_TIM_RGB] = (delay_from[DELAY_TIM_RGB] < delay_to[DELAY_TIM_RGB]);
    delay_step[DELAY_TIM_RGB] = RGB_STEP;
    
#endif
    
    RINGST_W(r_index) = _w;
    RINGST_R(r_index) = _r;
    RINGST_G(r_index) = _g;
    RINGST_B(r_index) = _b;
    
#ifdef GRADUAL_RGB
    // Smoothly change RGBW - set timer
    delay_timer[DELAY_TIM_RGB] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_RGB] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_RGB] = ChangeDeviceWRGB;
    delay_tag[DELAY_TIM_RGB] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_RGB, 1); // Enable RGB Dimmer operation
#else    
    uint32_t newValue = _w;
    newValue <<= 8;
    newValue += _r;
    newValue <<= 8;
    newValue += _g;
    newValue <<= 8;
    newValue += _b;
    ChangeDeviceWRGB(newValue, _ring);
#endif
    
    gIsChanged = TRUE;
    return TRUE;
  }
  
#else
  
  if( _w != DEVST_W || _r != DEVST_R || _g != DEVST_G || _b != DEVST_B ) {
#ifdef GRADUAL_RGB    
    // Smoothly change RGBW - set parameters
    delay_from[DELAY_TIM_RGB] = DEVST_W;
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += DEVST_R;
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += DEVST_G;
    delay_from[DELAY_TIM_RGB] <<= 8;
    delay_from[DELAY_TIM_RGB] += DEVST_B;
    delay_to[DELAY_TIM_RGB] = _w;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _r;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _g;
    delay_to[DELAY_TIM_RGB] <<= 8;
    delay_to[DELAY_TIM_RGB] += _b;
    delay_up[DELAY_TIM_RGB] = (delay_from[DELAY_TIM_RGB] < delay_to[DELAY_TIM_RGB]);
    delay_step[DELAY_TIM_RGB] = RGB_STEP;
#endif
    
    DEVST_W = _w;
    DEVST_R = _r;
    DEVST_G = _g;
    DEVST_B = _b;
    
#ifdef GRADUAL_RGB
    // Smoothly change RGBW - set timer
    delay_timer[DELAY_TIM_RGB] = 0x1FF;  // about 5ms
    delay_tick[DELAY_TIM_RGB] = 0;      // execute next step right away
    delay_handler[DELAY_TIM_RGB] = ChangeDeviceWRGB;
    delay_tag[DELAY_TIM_RGB] = _ring;
    BF_SET(delay_func, 1, DELAY_TIM_RGB, 1); // Enable RGB Dimmer operation
#else
    uint32_t newValue = _w;
    newValue <<= 8;
    newValue += _r;
    newValue <<= 8;
    newValue += _g;
    newValue <<= 8;
    newValue += _b;
    ChangeDeviceWRGB(newValue, _ring);
#endif
    
    gIsChanged = TRUE;
    return TRUE;
  }
  
#endif
  
  return FALSE;
}

// Gradually change on/off, brightness and cct in one
bool SetDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct, uint8_t _ring) {
#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  uint8_t oldSW = RINGST_OnOff(r_index);
  uint8_t oldBR = RINGST_Bright(r_index);
  uint8_t oldCCT = RINGST_WarmCold(r_index);
  
  if( _ring == RING_ID_ALL ) {
    RINGST_OnOff(0) = _sw;
    RINGST_OnOff(1) = _sw;
    RINGST_OnOff(2) = _sw;
    RINGST_Bright(0) = _br;
    RINGST_Bright(1) = _br;
    RINGST_Bright(2) = _br;
    RINGST_WarmCold(0) = _cct;
    RINGST_WarmCold(1) = _cct;
    RINGST_WarmCold(2) = _cct;
  } else {
    RINGST_OnOff(r_index) = _sw;
    RINGST_Bright(r_index) = _br;
    RINGST_WarmCold(r_index) = _cct;
  }
  
  if( _sw != oldSW ) {
    RINGST_OnOff(r_index) = oldSW;
    SetDeviceOnOff(_sw, _ring);
    return TRUE;
  }
  
  if( _br != oldBR ) {
    RINGST_Bright(r_index) = oldBR;
    SetDeviceBrightness(_br, _ring);
    return TRUE;    
  }
  
  RINGST_WarmCold(r_index) = oldCCT;
  return SetDeviceCCT(_cct, _ring);

#else  
  
  if( _sw != DEVST_OnOff ) {
    DEVST_Bright = _br;
    DEVST_WarmCold = _cct;
    SetDeviceOnOff(_sw, _ring);
    return TRUE;
  }
  
  if( _br != DEVST_Bright ) {
    DEVST_WarmCold = _cct;
    SetDeviceBrightness(_br, _ring);
    return TRUE;    
  }
  
  return SetDeviceCCT(_cct, _ring);
  
#endif  
}

// Gradually change on/off, brightness and rgbw in one
bool SetDeviceHue(bool _sw, uint8_t _br, uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _ring) {
#ifdef RING_INDIVIDUAL_COLOR
  
  uint8_t r_index = (_ring == RING_ID_ALL ? 0 : _ring - 1);
  uint8_t oldSW = RINGST_OnOff(r_index);
  uint8_t oldBR = RINGST_Bright(r_index);
  uint8_t oldW = RINGST_W(r_index);
  uint8_t oldR = RINGST_R(r_index);
  uint8_t oldG = RINGST_G(r_index);
  uint8_t oldB = RINGST_B(r_index);
  
  if( _ring == RING_ID_ALL ) {
    RINGST_OnOff(0) = _sw;
    RINGST_OnOff(1) = _sw;
    RINGST_OnOff(2) = _sw;
    RINGST_Bright(0) = _br;
    RINGST_Bright(1) = _br;
    RINGST_Bright(2) = _br;
    RINGST_W(0) = _w;
    RINGST_W(1) = _w;
    RINGST_W(2) = _w;
    RINGST_R(0) = _r;
    RINGST_R(1) = _r;
    RINGST_R(2) = _r;
    RINGST_G(0) = _g;
    RINGST_G(1) = _g;
    RINGST_G(2) = _g;
    RINGST_B(0) = _b;
    RINGST_B(1) = _b;
    RINGST_B(2) = _b;
  } else {
    RINGST_OnOff(r_index) = _sw;
    RINGST_Bright(r_index) = _br;
    RINGST_W(r_index) = _w;
    RINGST_R(r_index) = _r;
    RINGST_G(r_index) = _g;
    RINGST_B(r_index) = _b;
  }
  
  if( _sw != oldSW ) {
    RINGST_OnOff(r_index) = oldSW;
    SetDeviceOnOff(_sw, _ring);
    return TRUE;
  }
  
  if( _br != oldBR ) {
    RINGST_Bright(r_index) = oldBR;
    SetDeviceBrightness(_br, _ring);
    return TRUE;    
  }

  RINGST_W(r_index) = oldW;
  RINGST_R(r_index) = oldR;
  RINGST_G(r_index) = oldG;
  RINGST_B(r_index) = oldB;
  return SetDeviceWRGB(_w, _r, _g, _b, _ring);

#else  
  
  if( _sw != DEVST_OnOff ) {
    DEVST_Bright = _br;
    DEVST_W = _w;
    DEVST_R = _r;
    DEVST_G = _g;
    DEVST_B = _b;
    SetDeviceOnOff(_sw, _ring);
    return TRUE;
  }
  
  if( _br != DEVST_Bright ) {
    DEVST_W = _w;
    DEVST_R = _r;
    DEVST_G = _g;
    DEVST_B = _b;
    SetDeviceBrightness(_br, _ring);
    return TRUE;    
  }
  
  return SetDeviceWRGB(_w, _r, _g, _b, _ring);
  
#endif  
}

bool isTimerCompleted(uint8_t _tmr) {
  bool bFinished;
  
  if( _tmr == DELAY_TIM_RGB ) {
    uint32_t from_value, to_value, new_value;
    uint8_t color1, color2;
    from_value = delay_from[DELAY_TIM_RGB];
    to_value = delay_to[DELAY_TIM_RGB];
    new_value = 0;
    for( uint8_t i = 0; i < 4; i++ ) {
      color1 = (from_value & 0xFF);
      color2 = (to_value & 0xFF);
      if( color1 > color2 ) {
        color1 -= delay_step[DELAY_TIM_RGB];
        if( color1 < color2 ) color1 = color2;
      } else if( color1 < color2 ) {
        color1 += delay_step[DELAY_TIM_RGB];
        if( color1 > color2 ) color1 = color2;
      }
      uint32_t temp = color1;
      temp <<= (i * 8);
      new_value += temp;
      from_value >>= 8;
      to_value >>= 8;
    }
    delay_from[DELAY_TIM_RGB] = new_value;
    bFinished = ( delay_from[_tmr] == delay_to[_tmr] );
  } else {
    if( delay_up[_tmr] ) {
      // Up
      delay_from[_tmr] += delay_step[_tmr];
      bFinished = ( delay_from[_tmr] >= delay_to[_tmr] );
    } else {
      // Down
      if( delay_from[_tmr] > delay_step[_tmr] ) {
        delay_from[_tmr] -= delay_step[_tmr];
        bFinished = FALSE;
      } else {
        bFinished = TRUE;
      }
    }
  }
  
  // Execute operation
  if( delay_handler[_tmr] ) {
    if( bFinished ) {
      // Completed - to the final status
      (*delay_handler[_tmr])(delay_to[_tmr], delay_tag[_tmr]);
    } else {
      // Progress
      (*delay_handler[_tmr])(delay_from[_tmr], delay_tag[_tmr]);
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
      break; // Only one time at a time
    }
  }
  
  return mutex; 
}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(pMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
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

INTERRUPT_HANDLER(UART2_RX_IRQHandler, 18)
{
  /* In order to detect unexpected events during development,
  it is recommended to set a breakpoint on the following instruction.
  */
  if(UART2_GetITStatus(UART2_IT_RXNE) == SET) {
    RX_TX_BUFF = UART2_ReceiveData8();
    
    USART_ReceiveDataBuf[Buff_Cnt++] = RX_TX_BUFF ;
    if(RX_TX_BUFF == '\n') {
      USART_FLAG = 1;
      UART2_ClearITPendingBit(UART2_IT_RXNE);
    }
  }
  //Buff_Cnt++;
}
