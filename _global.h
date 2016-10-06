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

#define CT_MIN_VALUE            2700
#define CT_MAX_VALUE            6500
#define CT_SCOPE                38    
#define CT_STEP                 ((CT_MAX_VALUE-CT_MIN_VALUE)/10)

#define UNIQUE_ID_LEN           8

typedef struct
{
  UC State                    :1;           // Component state
  UC BR                       :7;           // Brightness of white [0..100]
  US CCT                      :16;          // CCT (warm or cold) [2700..6500]
  UC R                        :8;           // Brightness of red
  UC G                        :8;           // Brightness of green
  UC B                        :8;           // Brightness of blue
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
  Hue_t ring1;
  Hue_t ring2;
  Hue_t ring3;
  char Organization[24];                    // Organization name
  char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC Reserved1                :6;           // Reserved bits
} Config_t;

extern Config_t gConfig;
extern bool gIsChanged;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern uint16_t pwm_Warm;
extern uint16_t pwm_Cold;

void UpdateNodeAddress(void);
void CCT2ColdWarm(uint32_t ucBright, uint32_t ucWarmCold);

#define DEVST_OnOff             gConfig.ring1.State
#define DEVST_Bright            gConfig.ring1.BR
#define DEVST_WarmCold          gConfig.ring1.CCT

#endif /* __GLOBAL_H */