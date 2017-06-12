#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "LightPwmDrv.h"
#include "xliNodeConfig.h"

uint8_t bMsgReady = 0;
bool bDelaySend = FALSE;

// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    msg.header.version_length = PROTOCOL_VERSION;
    msg.header.sender = gConfig.nodeID;
    msg.header.destination = _destination;
    msg.header.sensor = _sensor;
    msg.header.type = _type;
    miSetCommand(_command);
    miSetRequestAck(_enableAck);
    miSetAck(_isAck);
}

uint8_t ParseProtocol(){
  if( msg.header.destination != gConfig.nodeID && msg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = msg.header.sender;  // The original sender
  uint8_t _type = msg.header.type;
  uint8_t _sensor = msg.header.sensor;
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  
  bDelaySend = FALSE;
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( lv_nodeID == NODEID_GATEWAY || lv_nodeID == NODEID_DUMMY ) {
      } else {
        if( miGetLength() > 8 ) {
          // Verify _uniqueID        
          if(!isIdentityEqual(_uniqueID, msg.payload.data+8, UNIQUE_ID_LEN)) {
            return 0;
          }
        }
        gConfig.nodeID = lv_nodeID;
        memcpy(gConfig.NetworkID, msg.payload.data, sizeof(gConfig.NetworkID));
        gIsChanged = TRUE;
        GotNodeID();
        // Increase brightness to indicate ID required
        SetDeviceBrightness(DEFAULT_BRIGHTNESS + 10, RING_ID_ALL);
        Msg_DevBrightness(_sender, _sensor);
        return 1;
      }
    } else if( _type == I_REBOOT ) {
      if( _sensor == NODEID_GATEWAY ) {
        // Verify token
        //if(!gConfig.present || gConfig.token == msg.payload.uiValue) {
          // Soft reset
          WWDG->CR = 0x80;
        //}
        return 0;
      }
    } else if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
      case NCF_QUERY:
        // Inform controller with version & NCF data
        Msg_NodeConfigData(_sender);
        return 1;
        break;

      case NCF_DEV_EN_SDTM:
        gConfig.enSDTM = msg.payload.data[0];
        break;
        
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = msg.payload.data[0];
        break;
        
      case NCF_MAP_SENSOR:
        gConfig.senMap = msg.payload.data[0] + msg.payload.data[1] * 256;
        break;
        
      case NCF_MAP_FUNC:
        gConfig.funcMap = msg.payload.data[0] + msg.payload.data[1] * 256;
        break;

      case NCF_DATA_ALS_RANGE:
        gConfig.alsLevel[0] = msg.payload.data[0];
        gConfig.alsLevel[1] = msg.payload.data[1];
        if( gConfig.alsLevel[1] < gConfig.alsLevel[0] ) {
          gConfig.alsLevel[1] = gConfig.alsLevel[0];
        }
        break;

      case NCF_DATA_PIR_RANGE:
        gConfig.pirLevel[0] = msg.payload.data[0];
        gConfig.pirLevel[1] = msg.payload.data[1];
        break;
      }
      gIsChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }
    break;
    
  case C_PRESENTATION:
    if( _sensor == S_LIGHT ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = msg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        gIsChanged = TRUE;
        // Inform controller with latest status
        Msg_DevStatus(NODEID_GATEWAY, NODEID_MIN_REMOTE, RING_ID_ALL);
        GotPresented();
        return 1;
      }
    }
    break;
    
  case C_REQ:
    if( _needAck ) {
      if( _type == V_STATUS || _type == V_PERCENTAGE ) {
        Msg_DevBrightness(_sender, _sensor);
        return 1;
      } else if( _type == V_LEVEL ) { // CCT
        Msg_DevBrightness(_sender, _sensor);
        return 1;
      } else if( _type == V_RGBW ) { // Hue
        uint8_t _RingID = msg.payload.data[0];
        Msg_DevStatus(_sender, _sensor, _RingID);
        return 1;
      } else if( _type == V_DISTANCE ) { // Topology
        uint8_t _RingID = msg.payload.data[0];
        Msg_DevTopology(_sender, _sensor, _RingID);
        return 1;
      }
    }    
    break;
    
  case C_SET:
    if( _type == V_STATUS ) {
      if( !_isAck ) {
        // set main lamp(ID:1) power(V_STATUS:2) on/off
        bool _OnOff = (msg.payload.bValue == DEVICE_SW_TOGGLE ? DEVST_OnOff == DEVICE_SW_OFF : msg.payload.bValue == DEVICE_SW_ON);
        SetDeviceOnOff(_OnOff, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevBrightness(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_PERCENTAGE ) {
      if( !_isAck ) {
        // Get main lamp(ID:1) dimmer (V_PERCENTAGE:3)
        uint8_t _Brightness;
        gConfig.filter = 0;
        if( miGetLength() == 2 ) {
          switch( msg.payload.data[0] ) {
          case OPERATOR_ADD:
            _Brightness = DEVST_Bright + msg.payload.data[1];
            if( _Brightness > 100 ) _Brightness = 100;
            break;
          case OPERATOR_SUB:
            if(DEVST_Bright > msg.payload.data[1] + BR_MIN_VALUE) {
              _Brightness = DEVST_Bright - msg.payload.data[1];
            } else {
              _Brightness = BR_MIN_VALUE;
            }
            break;
          default:      // OPERATOR_SET
            _Brightness = msg.payload.data[1];
          }
        } else {
          _Brightness = msg.payload.bValue;
        }
        SetDeviceBrightness(_Brightness, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevBrightness(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_LEVEL ) { // CCT
      if( !_isAck ) {
        // Get main lamp(ID:1) CCT V_LEVEL
        uint16_t _CCTValue;
        gConfig.filter = 0;
        if( miGetLength() == 3 ) {
          uint16_t _deltaValue = msg.payload.data[2] * 256 + msg.payload.data[1];
          switch( msg.payload.data[0] ) {
          case OPERATOR_ADD:
            _CCTValue = DEVST_WarmCold + _deltaValue;
            if( _CCTValue > CT_MAX_VALUE ) _CCTValue = CT_MAX_VALUE;
            break;
          case OPERATOR_SUB:
            if(DEVST_WarmCold > _deltaValue + CT_MIN_VALUE) {
              _CCTValue = DEVST_WarmCold - _deltaValue;
            } else {
              _CCTValue = CT_MIN_VALUE;
            }
            break;
          default:      // OPERATOR_SET
            _CCTValue = _deltaValue;
          }
        } else {
          _CCTValue = msg.payload.data[1] * 256 + msg.payload.data[0];
        }
        SetDeviceCCT(_CCTValue, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevCCT(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_RGBW ) { // RGBW
      if( !_isAck ) {
        // Get main lamp(ID:1) RGBW
        gConfig.filter = 0;
        uint8_t _RingID = msg.payload.data[0];
        if( _RingID > MAX_RING_NUM ) _RingID = RING_ID_ALL;
        uint8_t r_index = (_RingID == RING_ID_ALL ? 0 : _RingID - 1);
        
        bool _OnOff = msg.payload.data[1];
        uint8_t _Brightness = msg.payload.data[2];
        if( IS_SUNNY(gConfig.type) ) {
          uint16_t _CCTValue = msg.payload.data[4] * 256 + msg.payload.data[3];
          if( _OnOff != RINGST_OnOff(r_index) || _Brightness != RINGST_Bright(r_index) || _CCTValue != RINGST_WarmCold(r_index) ) {
            SetDeviceStatus(_OnOff, _Brightness, _CCTValue, _RingID);
            gIsChanged = TRUE;
          }
        } else if( IS_RAINBOW(gConfig.type) || IS_MIRAGE(gConfig.type) ) {
          // Set RGBW
          if( _OnOff != RINGST_OnOff(r_index) || _Brightness != RINGST_Bright(r_index) || msg.payload.data[3] != RINGST_W(r_index) 
             || msg.payload.data[4] != RINGST_R(r_index) || msg.payload.data[5] != RINGST_G(r_index) || msg.payload.data[6] != RINGST_B(r_index) ) {
            SetDeviceHue(_OnOff, _Brightness, msg.payload.data[3], msg.payload.data[4], msg.payload.data[5], msg.payload.data[6], _RingID);
            gIsChanged = TRUE;
          }
        }
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevStatus(_sender, _sensor, _RingID);
          return 1;
        }          
      }
    } else if( _type == V_DISTANCE ) { // Topology
      if( !_isAck ) {
        // Get main lamp(ID:1) Length of threads
        uint8_t _RingID = msg.payload.data[0];
        if( IS_MIRAGE(gConfig.type) ) {
          // ToDo: set Topology
        }
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevTopology(_sender, _sensor, _RingID);
          return 1;
        }          
      }
    } else if( _type == V_VAR1 ) { // Special effect
      if( !_isAck ) {
        SetDeviceFilter(msg.payload.bValue);
        if( _needAck ) {
          bDelaySend = (msg.header.destination == BROADCAST_ADDRESS);
          Msg_DevFilter(_sender, _sensor);
          return 1;
        }
      }
    }
    break;
  }
  
  return 0;
}

void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf) {
  build(_to, _ncf, C_INTERNAL, I_CONFIG, 0, 1);

  msg.payload.data[0] = 1;      // OK
  miSetPayloadType(P_BYTE);
  miSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  msg.payload.data[payl_len++] = gConfig.version;
  msg.payload.data[payl_len++] = gConfig.type;
  msg.payload.data[payl_len++] = gConfig.senMap % 256;
  msg.payload.data[payl_len++] = gConfig.senMap / 256;
  msg.payload.data[payl_len++] = gConfig.funcMap % 256;
  msg.payload.data[payl_len++] = gConfig.funcMap / 256;
  msg.payload.data[payl_len++] = gConfig.alsLevel[0];
  msg.payload.data[payl_len++] = gConfig.alsLevel[1];
  msg.payload.data[payl_len++] = gConfig.pirLevel[0];
  msg.payload.data[payl_len++] = gConfig.pirLevel[1];
  msg.payload.data[payl_len++] = ((gConfig.filter << 4) | (gConfig.hasSiblingMCU << 3) | gConfig.rptTimes);
  msg.payload.data[payl_len++] = 0;     // Reservered
  
  miSetLength(payl_len);
  miSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for device
  build(BASESERVICE_ADDRESS, NODE_TYP_LAMP, C_INTERNAL, I_ID_REQUEST, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_LIGHT, C_PRESENTATION, gConfig.type, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device On/Off status message
void Msg_DevOnOff(uint8_t _to, uint8_t _dest) {
  build(_to, _dest, C_REQ, V_STATUS, 0, 1);
  miSetLength(1);
  miSetPayloadType(P_BYTE);
  msg.payload.bValue = DEVST_OnOff;
  bMsgReady = 1;
}

// Prepare device brightness message
void Msg_DevBrightness(uint8_t _to, uint8_t _dest) {
  build(_to, _dest, C_REQ, V_PERCENTAGE, 0, 1);
  miSetLength(2);
  miSetPayloadType(P_BYTE);
  msg.payload.data[0] = DEVST_OnOff;
  msg.payload.data[1] = DEVST_Bright;
  bMsgReady = 1;
}

// Prepare device CCT message
void Msg_DevCCT(uint8_t _to, uint8_t _dest) {
  build(_to, _dest, C_REQ, V_LEVEL, 0, 1);
  miSetLength(2);
  miSetPayloadType(P_UINT16);
  msg.payload.data[0] = DEVST_WarmCold % 256;
  msg.payload.data[1] = DEVST_WarmCold / 256;
  bMsgReady = 1;
}

// Prepare device status message
void Msg_DevStatus(uint8_t _to, uint8_t _dest, uint8_t _ring) {
  uint8_t payl_len, r_index;
  
  if( _ring > MAX_RING_NUM ) _ring = RING_ID_ALL;
  
  build(_to, _dest, C_REQ, V_RGBW, 0, 1);
  msg.payload.data[0] = 1;		// Success
  msg.payload.data[1] = gConfig.type;
  msg.payload.data[2] = gConfig.present;
  payl_len = 3;
  
  for( r_index = 0; r_index < MAX_RING_NUM; r_index++ ) {

#ifdef RING_INDIVIDUAL_COLOR
    // Specific ring or all rings?
    if( _ring != RING_ID_ALL ) {
        // Only once
        msg.payload.data[payl_len++] = _ring;   // 1, 2 or 3
        r_index = _ring - 1;
    } else {
      // Send one ring at a time
      msg.payload.data[payl_len++] = r_index + 1;
    }
#else
    _ring = 1;      // Use the first ring presenting all rings
    msg.payload.data[payl_len++] = RING_ID_ALL;
#endif
    
    msg.payload.data[payl_len++] = RINGST_OnOff(r_index);
    msg.payload.data[payl_len++] = RINGST_Bright(r_index);
    if( IS_SUNNY(gConfig.type) ) {
      msg.payload.data[payl_len++] = (uint8_t)(RINGST_WarmCold(r_index) % 256);
      msg.payload.data[payl_len++] = (uint8_t)(RINGST_WarmCold(r_index) / 256);
    } else if( IS_RAINBOW(gConfig.type) || IS_MIRAGE(gConfig.type) ) {
      msg.payload.data[payl_len++] = RINGST_W(r_index);
      msg.payload.data[payl_len++] = RINGST_R(r_index);
      msg.payload.data[payl_len++] = RINGST_G(r_index);
      msg.payload.data[payl_len++] = RINGST_B(r_index);
    }
    
    // Specific ring or all rings?
    if( _ring != RING_ID_ALL ) {
      break;
    }    
  }
  
  if( payl_len > MAX_PAYLOAD ) {
    // Should break the message into chunks
    payl_len = MAX_PAYLOAD;
  }
  
  miSetLength(payl_len);
  miSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

// Prepare topology message
void Msg_DevTopology(uint8_t _to, uint8_t _dest, uint8_t _ring) {
  uint8_t payl_len, r_index;
  
  if( _ring > MAX_RING_NUM ) _ring = RING_ID_ALL;
  
  build(_to, _dest, C_REQ, V_DISTANCE, 0, 1);

  msg.payload.data[0] = IS_MIRAGE(gConfig.type);        // Success
  msg.payload.data[1] = gConfig.type;
  msg.payload.data[2] = gConfig.present;
  payl_len = 3;
  
  if( msg.payload.data[0] ) {
    for( r_index = 0; r_index < MAX_RING_NUM; r_index++ ) {
      // Specific ring or all rings?
      if( _ring != RING_ID_ALL ) {
        // Only once
        msg.payload.data[payl_len++] = _ring;   // 1, 2 or 3
        r_index = _ring - 1;
      } else {
        // Send one ring at a time
        msg.payload.data[payl_len++] = r_index + 1;
      }
      
      msg.payload.data[payl_len++] = RINGST_L1(r_index);
      msg.payload.data[payl_len++] = RINGST_L2(r_index);
      msg.payload.data[payl_len++] = RINGST_L3(r_index);
      
      // Specific ring or all rings?
      if( _ring != RING_ID_ALL ) {
        break;
      }    
    }
  }
    
  if( payl_len > MAX_PAYLOAD ) {
    // Should break the message into chunks
    payl_len = MAX_PAYLOAD;
  }
  
  miSetLength(payl_len);
  miSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

// Prepare device filter message
void Msg_DevFilter(uint8_t _to, uint8_t _dest) {
  build(_to, _dest, C_REQ, V_VAR1, 0, 1);
  miSetLength(1);
  miSetPayloadType(P_BYTE);
  msg.payload.data[0] = gConfig.filter;
  bMsgReady = 1;
}

#ifdef EN_SENSOR_ALS
// Prepare ALS message
void Msg_SenALS(uint8_t _value) {
  build(NODEID_GATEWAY, S_LIGHT_LEVEL, C_PRESENTATION, V_LIGHT_LEVEL, 0, 0);
  miSetPayloadType(P_BYTE);
  miSetLength(1);
  msg.payload.data[0] = _value;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_PIR
// Prepare PIR message
void Msg_SenPIR(bool _sw) {
  build(NODEID_GATEWAY, S_IR, C_PRESENTATION, V_STATUS, 0, 0);
  miSetPayloadType(P_BYTE);
  miSetLength(1);
  msg.payload.data[0] = _sw;
  bMsgReady = 1;
}
#endif
  
#ifdef EN_SENSOR_PM25
// Prepare PM2.5 message
void Msg_SenPM25(uint16_t _value) {
  build(NODEID_GATEWAY, S_DUST, C_PRESENTATION, V_LEVEL, 0, 0);
  miSetPayloadType(P_UINT16);
  miSetLength(2);
  msg.payload.data[0] = _value % 256;
  msg.payload.data[1] = _value / 256;
  bMsgReady = 1;  
}
#endif