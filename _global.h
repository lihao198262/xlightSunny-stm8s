#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8s.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8s_conf.h"

/* Exported types ------------------------------------------------------------*/
// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

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
#define DEFAULT_BRIGHTNESS      65
#define BRIGHTNESS_STEP         1
#define CCT_STEP                50

// Whether allow individual color control of ring
/// Uncomment this line only if hardware supports
//#define RING_INDIVIDUAL_COLOR

// Device (lamp) type
#define MAX_RING_NUM            3
typedef enum
{
  devtypUnknown = 0,
  devtypCRing3,     // Color ring - Rainbow
  devtypCRing2,
  devtypCRing1,
  devtypWRing3,     // White ring - Sunny
  devtypWRing2,
  devtypWRing1,
  devtypMRing3 = 8, // Color & Motion ring - Mirage
  devtypMRing2,
  devtypMRing1,
  devtypDummy = 255
} devicetype_t;


// Remote type
typedef enum
{
  remotetypUnknown = 0,
  remotetypRFSimply,
  remotetypRFStandard,
  remotetypRFEnhanced,
  remotetypDummy
} remotetype_t;

typedef struct
{
  UC State                    :1;           // Component state
  UC BR                       :7;           // Brightness of white [0..100]
  US CCT                      :16;          // CCT (warm or cold) [2700..6500]
  UC R                        :8;           // Brightness of red
  UC G                        :8;           // Brightness of green
  UC B                        :8;           // Brightness of blue
  UC L1                       :8;           // Length of thread 1
  UC L2                       :8;           // Length of thread 2
  UC L3                       :8;           // Length of thread 3
} Hue_t;

typedef struct
{
  UC version                  :8;           // Data version, other than 0xFF
  UC nodeID;                                // Remote Node ID
  UC NetworkID[6];
  UC present                  :1;           // 0 - not present; 1 - present
  UC reserved                 :7;
  UC type;                                  // Type of lamp
  US token;
  Hue_t ring[MAX_RING_NUM];
  char Organization[24];                    // Organization name
  char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC hasSiblingMCU            :1;           // Whether sibling MCU presents
  UC Reserved1                :5;           // Reserved bits
} Config_t;

extern Config_t gConfig;
extern bool gIsChanged;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern uint16_t pwm_Warm;
extern uint16_t pwm_Cold;

void UpdateNodeAddress(void);
void CCT2ColdWarm(uint32_t ucBright, uint32_t ucWarmCold);
void ChangeDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct, uint8_t _ring);
bool SetDeviceOnOff(bool _sw, uint8_t _ring);
bool SetDeviceBrightness(uint8_t _br, uint8_t _ring);
bool SetDeviceCCT(uint16_t _cct, uint8_t _ring);
bool SetDeviceStatus(bool _sw, uint8_t _br, uint16_t _cct, uint8_t _ring);
uint8_t idleProcess();

// All rings or the first ring
#define DEVST_OnOff             gConfig.ring[0].State
#define DEVST_Bright            gConfig.ring[0].BR
#define DEVST_WarmCold          gConfig.ring[0].CCT
#define DEVST_R                 gConfig.ring[0].R
#define DEVST_G                 gConfig.ring[0].G
#define DEVST_B                 gConfig.ring[0].B

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
#define RINGST_L1(rid)          gConfig.ring[(rid)].L1
#define RINGST_L2(rid)          gConfig.ring[(rid)].L2
#define RINGST_L3(rid)          gConfig.ring[(rid)].L3

#define IS_SUNNY(DevType)           ((DevType) >= devtypWRing3 && (DevType) <= devtypWRing1)
#define IS_RAINBOW(DevType)         ((DevType) >= devtypCRing3 && (DevType) <= devtypCRing1)
#define IS_MIRAGE(DevType)          ((DevType) >= devtypMRing3 && (DevType) <= devtypMRing1)
#define IS_VALID_REMOTE(DevType)    ((DevType) >= remotetypRFSimply && (DevType) <= remotetypRFEnhanced)

#define IS_NOT_DEVICE_NODEID(nID)  ((nID < NODEID_MIN_DEVCIE || nID > NODEID_MAX_DEVCIE) && nID != NODEID_MAINDEVICE)
#define IS_NOT_REMOTE_NODEID(nID)  (nID < NODEID_MIN_REMOTE || nID > NODEID_MAX_REMOTE)

#endif /* __GLOBAL_H */