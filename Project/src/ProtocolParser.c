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
    sndMsg.header.version_length = PROTOCOL_VERSION;
    sndMsg.header.sender = gConfig.nodeID;
    sndMsg.header.destination = _destination;
    sndMsg.header.sensor = _sensor;
    sndMsg.header.type = _type;
    moSetCommand(_command);
    moSetRequestAck(_enableAck);
    moSetAck(_isAck);
}

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  bool _specificNode = ((rcvMsg.header.destination == gConfig.nodeID ) && !IS_NOT_DEVICE_NODEID(gConfig.nodeID));
  
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
          if(!isIdentityEqual(_uniqueID, rcvMsg.payload.data+8, UNIQUE_ID_LEN)) {
            return 0;
          }
        }
        gConfig.nodeID = lv_nodeID;
        memcpy(gConfig.NetworkID, rcvMsg.payload.data, sizeof(gConfig.NetworkID));
        gIsChanged = TRUE;
        GotNodeID();
        // Increase brightness to indicate ID required
        SetDeviceBrightness(DEFAULT_BRIGHTNESS + 10, RING_ID_ALL);
        Msg_DevBrightness(_sender);
        return 1;
      }
    } else if( _type == I_REBOOT ) {
      if( IS_MINE_SUBID(_sensor) || _specificNode ) {
        // Verify token
        //if(!gConfig.present || gConfig.token == rcvMsg.payload.uiValue) {
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

      case NCF_DEV_SET_SUBID:
        if( _specificNode )
          gConfig.subID = rcvMsg.payload.data[0];
        break;

      case NCF_DEV_EN_SDTM:
        if( _specificNode )
          gConfig.enSDTM = rcvMsg.payload.data[0];
        break;
        
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = rcvMsg.payload.data[0];
        break;
        
      case NCF_MAP_SENSOR:
        gConfig.senMap = rcvMsg.payload.data[0] + rcvMsg.payload.data[1] * 256;
        break;
        
      case NCF_MAP_FUNC:
        gConfig.funcMap = rcvMsg.payload.data[0] + rcvMsg.payload.data[1] * 256;
        break;

      case NCF_DATA_ALS_RANGE:
        gConfig.alsLevel[0] = rcvMsg.payload.data[0];
        gConfig.alsLevel[1] = rcvMsg.payload.data[1];
        if( gConfig.alsLevel[1] < gConfig.alsLevel[0] ) {
          gConfig.alsLevel[1] = gConfig.alsLevel[0];
        }
        break;

      case NCF_DATA_PIR_RANGE:
        gConfig.pirLevel[0] = rcvMsg.payload.data[0];
        gConfig.pirLevel[1] = rcvMsg.payload.data[1];
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
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        GotPresented();
        gIsChanged = TRUE;
        // Inform controller with latest status
        Msg_DevStatus(NODEID_GATEWAY, RING_ID_ALL);
        return 1;
      }
    }
    break;
    
  case C_REQ:
    if( _needAck ) {
      if( IS_MINE_SUBID(_sensor) || _specificNode ) {
        if( _type == V_STATUS || _type == V_PERCENTAGE ) {
          Msg_DevBrightness(_sender);
          return 1;
        } else if( _type == V_LEVEL ) { // CCT
          Msg_DevBrightness(_sender);
          return 1;
        } else if( _type == V_RGBW ) { // Hue
          uint8_t _RingID = rcvMsg.payload.data[0];
          Msg_DevStatus(_sender, _RingID);
          return 1;
        } else if( _type == V_DISTANCE ) { // Topology
          uint8_t _RingID = rcvMsg.payload.data[0];
          Msg_DevTopology(_sender, _RingID);
          return 1;
        }
      }
    }    
    break;
    
  case C_SET:
    if( (IS_MINE_SUBID(_sensor) || _specificNode) && !_isAck ) {
      if( _type == V_STATUS ) {
        // set main lamp(ID:1) power(V_STATUS:2) on/off
        bool _OnOff = (rcvMsg.payload.bValue == DEVICE_SW_TOGGLE ? DEVST_OnOff == DEVICE_SW_OFF : rcvMsg.payload.bValue == DEVICE_SW_ON);
        SetDeviceOnOff(_OnOff, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevBrightness(_sender);
          return 1;
        }
      } else if( _type == V_PERCENTAGE ) {
        // Get main lamp(ID:1) dimmer (V_PERCENTAGE:3)
        uint8_t _Brightness;
        gConfig.filter = 0;
        if( miGetLength() == 2 ) {
          switch( rcvMsg.payload.data[0] ) {
          case OPERATOR_ADD:
            _Brightness = DEVST_Bright + rcvMsg.payload.data[1];
            if( _Brightness > 100 ) _Brightness = 100;
            break;
          case OPERATOR_SUB:
            if(DEVST_Bright > rcvMsg.payload.data[1] + BR_MIN_VALUE) {
              _Brightness = DEVST_Bright - rcvMsg.payload.data[1];
            } else {
              _Brightness = BR_MIN_VALUE;
            }
            break;
          default:      // OPERATOR_SET
            _Brightness = rcvMsg.payload.data[1];
          }
        } else {
          _Brightness = rcvMsg.payload.bValue;
        }
        SetDeviceBrightness(_Brightness, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevBrightness(_sender);
          return 1;
        }
      } else if( _type == V_LEVEL ) { // CCT
        // Get main lamp(ID:1) CCT V_LEVEL
        uint16_t _CCTValue;
        gConfig.filter = 0;
        if( miGetLength() == 3 ) {
          uint16_t _deltaValue = rcvMsg.payload.data[2] * 256 + rcvMsg.payload.data[1];
          switch( rcvMsg.payload.data[0] ) {
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
          _CCTValue = rcvMsg.payload.data[1] * 256 + rcvMsg.payload.data[0];
        }
        SetDeviceCCT(_CCTValue, RING_ID_ALL);
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevCCT(_sender);
          return 1;
        }
      } else if( _type == V_RGBW ) { // RGBW
        // Get main lamp(ID:1) RGBW
        gConfig.filter = 0;
        uint8_t _RingID = rcvMsg.payload.data[0];
        if( _RingID > MAX_RING_NUM ) _RingID = RING_ID_ALL;
        uint8_t r_index = (_RingID == RING_ID_ALL ? 0 : _RingID - 1);
        
        bool _OnOff = rcvMsg.payload.data[1];
        uint8_t _Brightness = rcvMsg.payload.data[2];
        if( IS_SUNNY(gConfig.type) ) {
          uint16_t _CCTValue = rcvMsg.payload.data[4] * 256 + rcvMsg.payload.data[3];
          if( _OnOff != RINGST_OnOff(r_index) || _Brightness != RINGST_Bright(r_index) || _CCTValue != RINGST_WarmCold(r_index) ) {
            SetDeviceStatus(_OnOff, _Brightness, _CCTValue, _RingID);
            gIsChanged = TRUE;
          }
        } else if( IS_RAINBOW(gConfig.type) || IS_MIRAGE(gConfig.type) ) {
          // Set RGBW
          if( _OnOff != RINGST_OnOff(r_index) || _Brightness != RINGST_Bright(r_index) || rcvMsg.payload.data[3] != RINGST_W(r_index) 
             || rcvMsg.payload.data[4] != RINGST_R(r_index) || rcvMsg.payload.data[5] != RINGST_G(r_index) || rcvMsg.payload.data[6] != RINGST_B(r_index) ) {
               SetDeviceHue(_OnOff, _Brightness, rcvMsg.payload.data[3], rcvMsg.payload.data[4], rcvMsg.payload.data[5], rcvMsg.payload.data[6], _RingID);
               gIsChanged = TRUE;
             }
        }
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevStatus(_sender, _RingID);
          return 1;
        }
      } else if( _type == V_DISTANCE ) { // Topology
        // Get main lamp(ID:1) Length of threads
        uint8_t _RingID = rcvMsg.payload.data[0];
        if( IS_MIRAGE(gConfig.type) ) {
          // ToDo: set Topology
        }
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevTopology(_sender, _RingID);
          return 1;
        }          
      } else if( _type == V_VAR1 ) { // Special effect
        SetDeviceFilter(rcvMsg.payload.bValue);
        if( _needAck ) {
          bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
          Msg_DevFilter(_sender);
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

  sndMsg.payload.data[0] = 1;      // OK
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.senMap % 256;
  sndMsg.payload.data[payl_len++] = gConfig.senMap / 256;
  sndMsg.payload.data[payl_len++] = gConfig.funcMap % 256;
  sndMsg.payload.data[payl_len++] = gConfig.funcMap / 256;
  sndMsg.payload.data[payl_len++] = gConfig.alsLevel[0];
  sndMsg.payload.data[payl_len++] = gConfig.alsLevel[1];
  sndMsg.payload.data[payl_len++] = gConfig.pirLevel[0];
  sndMsg.payload.data[payl_len++] = gConfig.pirLevel[1];
  sndMsg.payload.data[payl_len++] = ((gConfig.filter << 4) | (gConfig.hasSiblingMCU << 3) | gConfig.rptTimes);
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for device
  build(BASESERVICE_ADDRESS, NODE_TYP_LAMP, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_LIGHT, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device On/Off status message
void Msg_DevOnOff(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_STATUS, 0, 1);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = DEVST_OnOff;
  bMsgReady = 1;
}

// Prepare device brightness message
void Msg_DevBrightness(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_PERCENTAGE, 0, 1);
  moSetLength(2);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.data[0] = DEVST_OnOff;
  sndMsg.payload.data[1] = DEVST_Bright;
  bMsgReady = 1;
}

// Prepare device CCT message
void Msg_DevCCT(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_LEVEL, 0, 1);
  moSetLength(2);
  moSetPayloadType(P_UINT16);
  sndMsg.payload.data[0] = DEVST_WarmCold % 256;
  sndMsg.payload.data[1] = DEVST_WarmCold / 256;
  bMsgReady = 1;
}

// Prepare device status message
void Msg_DevStatus(uint8_t _to, uint8_t _ring) {
  uint8_t payl_len, r_index;
  
  if( _ring > MAX_RING_NUM ) _ring = RING_ID_ALL;
  
  build(_to, gConfig.subID, C_REQ, V_RGBW, 0, 1);
  sndMsg.payload.data[0] = 1;		// Success
  sndMsg.payload.data[1] = gConfig.type;
  sndMsg.payload.data[2] = gConfig.present;
  payl_len = 3;
  
  for( r_index = 0; r_index < MAX_RING_NUM; r_index++ ) {

#ifdef RING_INDIVIDUAL_COLOR
    // Specific ring or all rings?
    if( _ring != RING_ID_ALL ) {
        // Only once
        sndMsg.payload.data[payl_len++] = _ring;   // 1, 2 or 3
        r_index = _ring - 1;
    } else {
      // Send one ring at a time
      sndMsg.payload.data[payl_len++] = r_index + 1;
    }
#else
    _ring = 1;      // Use the first ring presenting all rings
    sndMsg.payload.data[payl_len++] = RING_ID_ALL;
#endif
    
    sndMsg.payload.data[payl_len++] = RINGST_OnOff(r_index);
    sndMsg.payload.data[payl_len++] = RINGST_Bright(r_index);
    if( IS_SUNNY(gConfig.type) ) {
      sndMsg.payload.data[payl_len++] = (uint8_t)(RINGST_WarmCold(r_index) % 256);
      sndMsg.payload.data[payl_len++] = (uint8_t)(RINGST_WarmCold(r_index) / 256);
    } else if( IS_RAINBOW(gConfig.type) || IS_MIRAGE(gConfig.type) ) {
      sndMsg.payload.data[payl_len++] = RINGST_W(r_index);
      sndMsg.payload.data[payl_len++] = RINGST_R(r_index);
      sndMsg.payload.data[payl_len++] = RINGST_G(r_index);
      sndMsg.payload.data[payl_len++] = RINGST_B(r_index);
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
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

// Prepare topology message
void Msg_DevTopology(uint8_t _to, uint8_t _ring) {
  uint8_t payl_len, r_index;
  
  if( _ring > MAX_RING_NUM ) _ring = RING_ID_ALL;
  
  build(_to, gConfig.subID, C_REQ, V_DISTANCE, 0, 1);

  sndMsg.payload.data[0] = IS_MIRAGE(gConfig.type);        // Success
  sndMsg.payload.data[1] = gConfig.type;
  sndMsg.payload.data[2] = gConfig.present;
  payl_len = 3;
  
  if( sndMsg.payload.data[0] ) {
    for( r_index = 0; r_index < MAX_RING_NUM; r_index++ ) {
      // Specific ring or all rings?
      if( _ring != RING_ID_ALL ) {
        // Only once
        sndMsg.payload.data[payl_len++] = _ring;   // 1, 2 or 3
        r_index = _ring - 1;
      } else {
        // Send one ring at a time
        sndMsg.payload.data[payl_len++] = r_index + 1;
      }
      
      sndMsg.payload.data[payl_len++] = RINGST_L1(r_index);
      sndMsg.payload.data[payl_len++] = RINGST_L2(r_index);
      sndMsg.payload.data[payl_len++] = RINGST_L3(r_index);
      
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
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

// Prepare device filter message
void Msg_DevFilter(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_VAR1, 0, 1);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.data[0] = gConfig.filter;
  bMsgReady = 1;
}

#ifdef EN_SENSOR_ALS
// Prepare ALS message
void Msg_SenALS(uint8_t _value) {
  build(NODEID_GATEWAY, S_LIGHT_LEVEL, C_PRESENTATION, V_LIGHT_LEVEL, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _value;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_PIR
// Prepare PIR message
void Msg_SenPIR(bool _sw) {
  build(NODEID_GATEWAY, S_IR, C_PRESENTATION, V_STATUS, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _sw;
  bMsgReady = 1;
}
#endif
  
#ifdef EN_SENSOR_PM25
// Prepare PM2.5 message
void Msg_SenPM25(uint16_t _value) {
  build(NODEID_GATEWAY, S_DUST, C_PRESENTATION, V_LEVEL, 0, 0);
  moSetPayloadType(P_UINT16);
  moSetLength(2);
  sndMsg.payload.data[0] = _value % 256;
  sndMsg.payload.data[1] = _value / 256;
  bMsgReady = 1;  
}
#endif