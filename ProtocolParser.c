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
      }
    }
    break;
  
  case C_REQ:
    if( _needAck ) {
      if( _type == V_STATUS ) {
        build(_sender, _sensor, C_REQ, V_STATUS, 0, 1);
        miSetLength(1);
        miSetPayloadType(P_BYTE);
        msg.payload.bValue = DEVST_OnOff;
        //msg.payload.data[0] = DEVST_OnOff;
        return 1;
      } else if( _type == V_PERCENTAGE ) {
        build(_sender, _sensor, C_REQ, V_PERCENTAGE, 0, 1);
        miSetLength(1);
        miSetPayloadType(P_BYTE);
        msg.payload.bValue = DEVST_Bright;
        return 1;
      } else if( _type == V_LEVEL ) { // CCT
        build(_sender, _sensor, C_REQ, V_LEVEL, 0, 1);
        miSetLength(2);
        miSetPayloadType(P_UINT16);
        //msg.payload.uiValue = DEVST_WarmCold;
        msg.payload.data[0] = DEVST_WarmCold % 256;
        msg.payload.data[1] = DEVST_WarmCold / 256;
        return 1;
      } else if( _type == V_RGBW ) {
        build(_sender, _sensor, C_REQ, V_RGBW, 0, 1);
        uint8_t payload[8];
        payload[0] = 1;		// Success
        payload[1] = gConfig.type;
        payload[2] = gConfig.present;
        payload[3] = DEVST_OnOff;
        payload[4] = DEVST_Bright;
        payload[5] = (uint8_t)(DEVST_WarmCold / 256);
        payload[6] = (uint8_t)(DEVST_WarmCold % 256);
        payload[7] = 0;
        miSetLength(8);
        miSetPayloadType(P_CUSTOM);
        memcpy(msg.payload.data, payload, 8);
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
          if( _OnOff != DEVST_OnOff ) {
            CCT2ColdWarm(_OnOff ? 100 : 0, DEVST_WarmCold);
            driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
            DEVST_OnOff = _OnOff;
            gIsChanged = TRUE;
          }
          if( _needAck ) {
            build(_sender, _sensor, C_SET, V_STATUS, 0, 1);
            miSetLength(1);
            miSetPayloadType(P_BYTE);
            msg.payload.bValue = DEVST_OnOff;
            //msg.payload.data[0] = DEVST_OnOff;
            return 1;
          }
      }
    } else if( _type == V_PERCENTAGE ) {
      if( !_isAck ) {
        // Get main lamp(ID:1) dimmer (V_PERCENTAGE:3)
        uint8_t _Brightness = msg.payload.bValue;
        sprintf(strOutput, "Got lights:%d dimmer %d msg", _sensor, _Brightness);
        if( _Brightness != DEVST_Bright ) {
          DEVST_Bright = _Brightness;
          DEVST_OnOff = (_Brightness > 0);
          gIsChanged = TRUE;
          CCT2ColdWarm(DEVST_Bright, DEVST_WarmCold);
          driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
        }
        if( _needAck ) {
          build(_sender, _sensor, C_SET, V_PERCENTAGE, 0, 1);
          miSetLength(1);
          miSetPayloadType(P_BYTE);
          msg.payload.bValue = _Brightness;
          //msg.payload.data[0] = _Brightness;
          return 1;
        }
      }
    } else if( _type == V_LEVEL ) { // CCT
      if( !_isAck ) {
        // Get main lamp(ID:1) CCT V_LEVEL
        //uint16_t _CCTValue = (uint16_t)msg.payload.uiValue;
        uint16_t _CCTValue = msg.payload.data[1] * 256 + msg.payload.data[0];
        sprintf(strOutput, "Got lights:%d CCT level %d msg", _sensor, _CCTValue);
        if( _CCTValue != DEVST_WarmCold ) {
          DEVST_WarmCold = _CCTValue;
          gIsChanged = TRUE;
          CCT2ColdWarm(DEVST_Bright, DEVST_WarmCold);
          driveColdWarmLightPwm(pwm_Cold, pwm_Warm);
        }
        if( _needAck ) {
          build(_sender, _sensor, C_SET, V_LEVEL, 0, 1);
          miSetLength(2);
          miSetPayloadType(P_UINT16);
          //msg.payload.uiValue = _CCTValue;
          msg.payload.data[0] = _CCTValue % 256;
          msg.payload.data[1] = _CCTValue / 256;
          return 1;
        }
       }
    }
    break;
  }
  
  return 0;
}

void Msg_Presentation() {
  build(NODEID_GATEWAY, gConfig.type, C_PRESENTATION, S_LIGHT, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
}
