#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "LightPwmDrv.h"

char strOutput[50];

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
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( lv_nodeID == NODEID_GATEWAY || lv_nodeID == NODEID_DUMMY ) {
      } else {
        sprintf(strOutput, "Get NodeId: %d, networkId: %X:%X:%X:%X:%X:%X:%X:%X", lv_nodeID, 
                msg.payload.data[0], msg.payload.data[1], msg.payload.data[2], msg.payload.data[3], 
                msg.payload.data[4], msg.payload.data[5], msg.payload.data[6], msg.payload.data[7]);
        gConfig.nodeID = lv_nodeID;
        gIsChanged = TRUE;
        memcpy(gConfig.NetworkID, msg.payload.data, sizeof(gConfig.NetworkID));
        UpdateNodeAddress();
        // I'm alive
        Msg_Presentation();
        return 1;
      }
    }    
    break;
    
  case C_PRESENTATION:
    if( _type == S_LIGHT ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = msg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        gIsChanged = TRUE;
        sprintf(strOutput, "Got Presentation Ack Node:%d token:%d", gConfig.nodeID, gConfig.token);
        // Inform controller with latest status
        Msg_DevStatus(NODEID_GATEWAY, NODEID_MIN_REMOTE);
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
      } else if( _type == V_RGBW ) {
        Msg_DevStatus(_sender, _sensor);
        return 1;
      }
    }    
    break;
    
  case C_SET:
    if( _type == V_STATUS ) {
      if( !_isAck ) {
        // set main lamp(ID:1) power(V_STATUS:2) on/off
        bool _OnOff = msg.payload.bValue;
        sprintf(strOutput, "Got lights:%d turn %s msg", _sensor, _OnOff ? "on" : "off");
        SetDeviceOnOff(_OnOff);
        if( _needAck ) {
          Msg_DevBrightness(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_PERCENTAGE ) {
      if( !_isAck ) {
        // Get main lamp(ID:1) dimmer (V_PERCENTAGE:3)
        uint8_t _Brightness = msg.payload.bValue;
        sprintf(strOutput, "Got lights:%d dimmer %d msg", _sensor, _Brightness);
        SetDeviceBrightness(_Brightness);
        if( _needAck ) {
          Msg_DevBrightness(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_LEVEL ) { // CCT
      if( !_isAck ) {
        // Get main lamp(ID:1) CCT V_LEVEL
        //uint16_t _CCTValue = (uint16_t)msg.payload.uiValue;
        uint16_t _CCTValue = msg.payload.data[1] * 256 + msg.payload.data[0];
        sprintf(strOutput, "Got lights:%d CCT level %d msg", _sensor, _CCTValue);
        SetDeviceCCT(_CCTValue);
        if( _needAck ) {
          Msg_DevCCT(_sender, _sensor);
          return 1;
        }
      }
    } else if( _type == V_RGBW ) { // RGBW
      if( !_isAck ) {
        // Get main lamp(ID:1) RGBW
        bool _OnOff = msg.payload.data[0];
        uint8_t _Brightness = msg.payload.data[1];
        if( IS_SUNNY(gConfig.type) ) {
          uint16_t _CCTValue = msg.payload.data[3] * 256 + msg.payload.data[2];
          if( _OnOff != DEVST_OnOff || _Brightness != DEVST_Bright || _CCTValue != DEVST_WarmCold ) {
            SetDeviceStatus(_OnOff, _Brightness, _CCTValue);
            gIsChanged = TRUE;
          }
        } else if( IS_RAINBOW(gConfig.type) ) {
          // ToDo: Set RGBW
        } else if( IS_MIRAGE(gConfig.type) ) {
          // ToDo: set RGBW and Topology
        }
        if( _needAck ) {
          Msg_DevStatus(_sender, _sensor);
          return 1;
        }          
      }
    }
    break;
  }
  
  return 0;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, gConfig.type, C_PRESENTATION, S_LIGHT, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
}

// Prepare device On/Off status message
void Msg_DevOnOff(uint8_t _to, uint8_t _dest) {
  build(_to, _dest, C_REQ, V_STATUS, 0, 1);
  miSetLength(1);
  miSetPayloadType(P_BYTE);
  msg.payload.bValue = DEVST_OnOff;
}

// Prepare device brightness message
void Msg_DevBrightness(uint8_t _to, uint8_t _dest) {
  uint8_t payl_len = 2;

  build(_to, _dest, C_REQ, V_PERCENTAGE, 0, 1);
  miSetLength(2);
  miSetPayloadType(P_BYTE);
  msg.payload.data[0] = DEVST_OnOff;
  msg.payload.data[1] = DEVST_Bright;
}

// Prepare device CCT message
void Msg_DevCCT(uint8_t _to, uint8_t _dest) {
  uint8_t payl_len = 2;

  build(_to, _dest, C_REQ, V_LEVEL, 0, 1);
  miSetLength(2);
  miSetPayloadType(P_UINT16);
  msg.payload.data[0] = DEVST_WarmCold % 256;;
  msg.payload.data[1] = DEVST_WarmCold / 256;  ;
}

// Prepare device status message
void Msg_DevStatus(uint8_t _to, uint8_t _dest) {
  uint8_t payl_len;

  build(_to, _dest, C_REQ, V_RGBW, 0, 1);
  msg.payload.data[0] = 1;		// Success
  msg.payload.data[1] = gConfig.type;
  msg.payload.data[2] = gConfig.present;
  msg.payload.data[3] = DEVST_OnOff;
  msg.payload.data[4] = DEVST_Bright;
  if( IS_SUNNY(gConfig.type) ) {
    msg.payload.data[5] = (uint8_t)(DEVST_WarmCold % 256);
    msg.payload.data[6] = (uint8_t)(DEVST_WarmCold / 256);
    msg.payload.data[7] = 0;
    payl_len = 8;
  } else if( IS_RAINBOW(gConfig.type) ) {
    msg.payload.data[5] = (uint8_t)(DEVST_WarmCold % 256);
    msg.payload.data[6] = gConfig.ring1.R;
    msg.payload.data[7] = gConfig.ring1.G;
    msg.payload.data[8] = gConfig.ring1.B;
    msg.payload.data[9] = 0;
    payl_len = 10;
  } else if( IS_MIRAGE(gConfig.type) ) {
    msg.payload.data[5] = (uint8_t)(DEVST_WarmCold % 256);
    msg.payload.data[6] = gConfig.ring1.R;
    msg.payload.data[7] = gConfig.ring1.G;
    msg.payload.data[8] = gConfig.ring1.B;
    msg.payload.data[9] = gConfig.ring1.L1;
    msg.payload.data[10] = gConfig.ring1.L2;
    msg.payload.data[11] = gConfig.ring1.L3;
    msg.payload.data[12] = gConfig.ring2.L1;
    msg.payload.data[13] = gConfig.ring2.L2;
    msg.payload.data[14] = gConfig.ring2.L3;
    msg.payload.data[15] = gConfig.ring3.L1;
    msg.payload.data[16] = gConfig.ring3.L2;
    msg.payload.data[17] = gConfig.ring3.L3;
    msg.payload.data[18] = 0;
    payl_len = MAX_PAYLOAD;
  }
  miSetLength(payl_len);
  miSetPayloadType(P_CUSTOM);
}
