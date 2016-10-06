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
uint8_t mutex;
uint8_t rx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t tx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint16_t pwm_Warm = 0;
uint16_t pwm_Cold = 0;

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
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  
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
      gConfig.type = 4;  // devtypWRing3
      gConfig.ring1.State = 1;
      gConfig.ring1.BR = 50;
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
    if( mutex > 0 ) return TRUE;
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
  
  pwm_Warm = (1000 - ucWarmCold)*ucBright/1000 ;

  pwm_Cold = ucWarmCold*ucBright/1000 ;
}

// Bring the lights to default status
void LightsInit(void)
{
  CCT2ColdWarm(DEVST_Bright, DEVST_WarmCold);
  driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
}

int main( void ) {
  uint8_t bMsgReady = 0;
  
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
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  
  // Bring the lights to default status
  LightsInit();
  
  Delay(0x1FFFF);   // about 3 sec
  
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
    while(!mutex);
    if (mutex == 1) {
      RF24L01_read_payload(pMsg, PLOAD_WIDTH);
      bMsgReady = ParseProtocol();
    }
    else {
      //Something happened
      Delay(0xFFF);
      continue;
    }
    
    // Save Config if Changed
    SaveConfig();
    
    // Send message
    if( bMsgReady == 1 ) {
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(pMsg, PLOAD_WIDTH);

      WaitMutex(0x1FFFF);
      if (mutex != 1) {
        //The transmission failed, Notes: mutex == 2 doesn't mean failed
        //It happens when rx address defers from tx address
        //asm("nop"); //Place a breakpoint here to see memory
      }
    }
  }
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
