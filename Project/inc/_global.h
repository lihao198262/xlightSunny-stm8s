#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8s.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8s_conf.h"

/* Exported types ------------------------------------------------------------*/
// Simple Direct Test
// Uncomment this line to work in Simple Direct Test Mode
//#define ENABLE_SDTM

// Include Sensors
/// Comment off line to disable sensor
//#define EN_SENSOR_ALS
//#define EN_SENSOR_MIC
//#define EN_SENSOR_PIR
//#define EN_SENSOR_DHT
//#define EN_SENSOR_PM25
//#define EN_SENSOR_MQ135
//#define EN_SENSOR_MQ2
//#define EN_SENSOR_MQ7

// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Switch value for set power command
#define DEVICE_SW_OFF               0       // Turn Off
#define DEVICE_SW_ON                1       // Turn On
#define DEVICE_SW_TOGGLE            2       // Toggle
#define DEVICE_SW_DUMMY             3       // Detail followed

// Update operator for set brightness & CCT command
#define OPERATOR_SET                0
#define OPERATOR_ADD                1
#define OPERATOR_SUB                2
#define OPERATOR_MUL                3
#define OPERATOR_DIV                4

// Filter (special effect)
#define FILTER_SP_EF_NONE           0
#define FILTER_SP_EF_BREATH         1       // Normal breathing light
#define FILTER_SP_EF_FAST_BREATH    2       // Fast breathing light
#define FILTER_SP_EF_FLORID         3       // Randomly altering color
#define FILTER_SP_EF_FAST_FLORID    4       // Fast randomly altering color

// Serial Command
#define UART_CMD_HELLO                  0
#define UART_CMD_CCT                    1
#define UART_CMD_RGBW                   2
#define UART_CMD_HELLO_ACK              16

// Node type
#define NODE_TYP_GW               'g'
#define NODE_TYP_LAMP             'l'
#define NODE_TYP_REMOTE           'r'
#define NODE_TYP_SYSTEM           's'
#define NODE_TYP_THIRDPARTY       't'

// NodeID Convention
#define NODEID_GATEWAY          0
#define NODEID_MAINDEVICE       1
#define NODEID_MIN_DEVCIE       8
#define NODEID_MAX_DEVCIE       63
#define NODEID_MIN_REMOTE       64
#define NODEID_MAX_REMOTE       127
#define NODEID_PROJECTOR        128
#define NODEID_KEYSIMULATOR     129
#define NODEID_SUPERSENSOR      130
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_RF_SCANNER       250
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF

#define BR_MIN_VALUE            1
#define CT_MIN_VALUE            2700
#define CT_MAX_VALUE            6500
#define CT_SCOPE                38    
#define CT_STEP                 ((CT_MAX_VALUE-CT_MIN_VALUE)/10)
#define LIGHT_PWM_THRESHOLD     5

#define UNIQUE_ID_LEN           8

// Delayed operation timers
#define DELAY_TIMERS            8
#define DELAY_TIM_ONOFF         0
#define DELAY_TIM_BR            1
#define DELAY_TIM_CCT           2
#define DELAY_TIM_RGB           3
#define DELAY_TIM_MSG           7

// Comment off the line to disable gradual brightness on or off
#define GRADUAL_ONOFF
#define GRADUAL_CCT
//#define GRADUAL_RGB
#define DEFAULT_BRIGHTNESS      65
#define BRIGHTNESS_STEP         1
#define MAX_STEP_TIMES          62
#define MAX_FASTSTEP_TIMES      20
#define CCT_STEP                50
#define RGB_STEP                3

// WATT regulation method
/// Option 0: no restriction
/// Option 1: percentage
/// Option 2: percentage + linear
/// Option 3: percentage + quadratic
/// Option 4: percentage + cubic function
/// Option 10: percentage + table
#define WATT_RM_NO_RESTRICTION          0
#define WATT_RM_PERCENTAGE              1
#define WATT_RM_LINEAR_PERCENTAGE       2
#define WATT_RM_QUADRATIC_PERCENTAGE    3
#define WATT_RM_CUBIC_PERCENTAGE        4
#define WATT_RM_TABLE_PERCENTAGE        10

// Keep alive message interval, around 6 seconds
#define RTE_TM_KEEP_ALIVE               500    // about 5s (500 * 10ms)
#define MAX_RF_FAILED_TIME              8      // Reset RF module when reach max failed times of sending
#define MAX_RF_RESET_TIME               3      // Reset Node when reach max times of RF module consecutive reset

// Whether allow individual color control of ring
/// Uncomment this line only if h       ardware supports
//#define RING_INDIVIDUAL_COLOR

// Device (lamp) type
#define MAX_RING_NUM            3
typedef enum
{
  devtypUnknown = 0,
  // Color ring - Rainbow
  devtypCRing3,
  devtypCRing2,
  devtypCBar,
  devtypCFrame,
  devtypCWave,
  devtypCRing1 = 31,

  // White ring - Sunny
  devtypWRing3 = 32,
  devtypWRing2,
  devtypWBar,
  devtypWFrame,
  devtypWWave,
  devtypWSquare60,      // 60 * 60
  devtypWPanel120_30,   // 120 * 30
  devtypWBlackboard,    // Blackboard lamp
  devtypWRing1 = 95,

  // Color & Motion ring - Mirage
  devtypMRing3 = 96,
  devtypMRing2,
  devtypMBar,
  devtypMFrame,
  devtypMWave,
  devtypMRing1 = 127,

  devtypDummy = 255
} devicetype_t;

typedef struct
{
  UC State                    :1;           // Component state
  UC BR                       :7;           // Brightness of white [0..100]
  US CCT                      :16;          // CCT (warm or cold) [2700..6500]
  UC R                        :8;           // Brightness of red
  UC G                        :8;           // Brightness of green
  UC B                        :8;           // Brightness of blue
  UC W                        :8;           // Brightness of white
  UC L1                       :8;           // Length of thread 1
  UC L2                       :8;           // Length of thread 2
  UC L3                       :8;           // Length of thread 3
} Hue_t;


// I_GET_NONCE sub-type
enum {
    SCANNER_PROBE = 0,
    SCANNER_SETUP_RF,           // by NodeID & SubID
    SCANNER_SETUPDEV_RF,        // by UniqueID
    
    SCANNER_GETCONFIG = 8,      // by NodeID & SubID
    SCANNER_SETCONFIG,
    SCANNER_GETDEV_CONFIG,      // by UniqueID
    SCANNER_SETDEV_CONFIG,
    
    SCANNER_TEST_NODE = 16,     // by NodeID & SubID
    SCANNER_TEST_DEVICE,        // by UniqueID
};

// Xlight Application Identification
#define XLA_VERSION               0x08
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM

#if XLA_VERSION > 0x07
#define XLA_MIN_VER_REQUIREMENT   0x08
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC filter                   :4;
  UC reserved0                :3;
  UC swTimes                  :4;           // On/Off times
  UC cntRFReset               :4;           // RF reset count
  Hue_t ring[MAX_RING_NUM];

  // Configurable parameters
  UC nodeID;                                // Node ID for this device
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC hasSiblingMCU            :1;           // Whether sibling MCU presents
  UC type;                                  // Type of lamp
  US token;
  UC wattOption               :4;           // 0..15, ref to WATT regulation method
  UC enAutoPowerTest          :1;           // power auto test mode
  UC reserved1                :3;
  US senMap                   :16;          // Sensor Map
  US funcMap                  :16;          // Function Map
  UC alsLevel[2];
  UC pirLevel[2];
} Config_t;
#else
#define XLA_MIN_VER_REQUIREMENT   0x03
typedef struct
{
  UC version                  :8;           // Data version, other than 0xFF
  UC nodeID;                                // Node ID for this device
  UC NetworkID[6];
  UC present                  :1;           // 0 - not present; 1 - present
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC filter                   :4;
  UC type;                                  // Type of lamp
  UC subID;                                 // SubID
  US token;
  Hue_t ring[MAX_RING_NUM];
  //char Organization[24];                    // Organization name
  //char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC hasSiblingMCU            :1;           // Whether sibling MCU presents
  UC swTimes                  :3;           // On/Off times
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  US senMap                   :16;          // Sensor Map
  US funcMap                  :16;          // Function Map
  UC alsLevel[2];
  UC pirLevel[2];
  UC cntRFReset               :4;           // RF reset count
  UC wattOption               :4;           // 0..15, ref to WATT regulation method
  UC rfChannel;                             // RF Channel: [0..127]
} Config_t;
#endif

extern Config_t gConfig;
extern bool gIsChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];
// off delay tick
extern int32_t offdelaytick;


bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
void GotNodeID();
void GotPresented();
void CCT2ColdWarm(uint32_t ucBright, uint32_t ucWarmCold);
bool SetDeviceOnOff(bool _sw, uint8_t _ring);
bool SetDeviceBrightness(uint8_t _br, uint8_t _ring);
bool SetDeviceCCT(uint16_t _cct, uint8_t _ring);
bool SetDeviceWRGB(uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _ring);
bool SetDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct, uint8_t _ring);
bool SetDeviceHue(bool _sw, uint8_t _br, uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _ring);
bool SetDeviceFilter(uint8_t _filter);
void tmrProcess();
void idleProcess();
void ChangeDeviceBR(uint32_t _br, uint8_t _ring);
void ResetNodeToRegister();
void printlog(uint8_t *pBuf);

#define SECOND_UNIT  1
#define MINUTE_UNIT  2
#define HOUR_UNIT    3

// All rings or the first ring
#define DEVST_OnOff             gConfig.ring[0].State
#define DEVST_Bright            gConfig.ring[0].BR
#define DEVST_WarmCold          gConfig.ring[0].CCT
#define DEVST_R                 gConfig.ring[0].R
#define DEVST_G                 gConfig.ring[0].G
#define DEVST_B                 gConfig.ring[0].B
#define DEVST_W                 gConfig.ring[0].W

#define RING_ID_ALL             0
#define RING_ID_1               1
#define RING_ID_2               2
#define RING_ID_3               3

// Specific ring
#define RINGST_OnOff(rid)       gConfig.ring[(rid)].State
#define RINGST_Bright(rid)      gConfig.ring[(rid)].BR
#define RINGST_WarmCold(rid)    gConfig.ring[(rid)].CCT
#define RINGST_R(rid)           gConfig.ring[(rid)].R
#define RINGST_G(rid)           gConfig.ring[(rid)].G
#define RINGST_B(rid)           gConfig.ring[(rid)].B
#define RINGST_W(rid)           gConfig.ring[(rid)].W
#define RINGST_L1(rid)          gConfig.ring[(rid)].L1
#define RINGST_L2(rid)          gConfig.ring[(rid)].L2
#define RINGST_L3(rid)          gConfig.ring[(rid)].L3

#define IS_SUNNY(DevType)           ((DevType) >= devtypWRing3 && (DevType) <= devtypWRing1)
#define IS_RAINBOW(DevType)         ((DevType) >= devtypCRing3 && (DevType) <= devtypCRing1)
#define IS_MIRAGE(DevType)          ((DevType) >= devtypMRing3 && (DevType) <= devtypMRing1)
#define IS_VALID_REMOTE(DevType)    ((DevType) >= remotetypRFSimply && (DevType) <= remotetypRFEnhanced)

#define IS_GROUP_NODEID(nID)       (nID >= NODEID_MIN_GROUP && nID <= NODEID_MAX_GROUP)
#define IS_NOT_DEVICE_NODEID(nID)  ((nID < NODEID_MIN_DEVCIE || nID > NODEID_MAX_DEVCIE) && nID != NODEID_MAINDEVICE)
#define IS_NOT_REMOTE_NODEID(nID)  (nID < NODEID_MIN_REMOTE || nID > NODEID_MAX_REMOTE)
#define IS_MINE_SUBID(nSID)        ((nSID) == 0 || ((nSID) & gConfig.subID))

//#define TEST

#ifdef TEST
#define     PB5_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_5)
#define     PB4_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_4)
#define     PB3_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_3)
#define     PB2_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_2)
#define     PB1_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_1)
#define     PB0_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_0)
#define     PD1_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_1)
#define     PD2_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_2)
#define     PD7_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_7)
#define     PC1_Low                GPIO_WriteLow(GPIOC , GPIO_PIN_1)
#define     PB5_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_5)
#define     PB4_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_4)
#define     PB3_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_3)
#define     PB2_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_2)
#define     PB1_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_1)
#define     PB0_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_0)
#define     PD1_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_1)
#define     PD2_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_2)
#define     PD7_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_7)
#define     PC1_High                GPIO_WriteHigh(GPIOC , GPIO_PIN_1)
#endif

#endif /* __GLOBAL_H */
