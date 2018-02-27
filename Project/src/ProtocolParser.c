#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "LightPwmDrv.h"
#include "xliNodeConfig.h"

uint8_t bMsgReady = 0;
bool bDelaySend = FALSE;

void MsgScanner_ProbeAck();
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid);
void Process_SetConfig(u8 _len);
void Process_SetDevConfig(u8 _len);
void Process_SetupRF(const UC *rfData,uint8_t rflen);

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid);
typedef struct
{
  uint8_t offset;
  uint8_t size;
  uint8_t byUniqueid;  // whether getconfig by uniqueid
}CfgBlock;
#define OUT_CFG_MESSAGE_LEN           16
CfgBlock out_cfg_msg_buf[OUT_CFG_MESSAGE_LEN];
u8 cfg_msg_out_buf_read_ptr = 0;
u8 cfg_msg_out_buf_write_ptr = 0;

bool AddCfgOutputBuf(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {  
  CfgBlock cfgblock;
  cfgblock.offset = offset;
  cfgblock.size = size;
  cfgblock.byUniqueid = isNeedUniqueid;
  out_cfg_msg_buf[cfg_msg_out_buf_write_ptr++] = cfgblock;
  cfg_msg_out_buf_write_ptr %= OUT_CFG_MESSAGE_LEN;
  return TRUE;
}

bool ProcessOutputCfgMsg() {
  // Send output Cfg msg
  while( cfg_msg_out_buf_read_ptr != cfg_msg_out_buf_write_ptr) {   
    CfgBlock cfgblock = out_cfg_msg_buf[cfg_msg_out_buf_read_ptr++];
    SendCfgBlock(cfgblock.offset,cfgblock.size,TRUE);
    cfg_msg_out_buf_read_ptr %= OUT_CFG_MESSAGE_LEN;
  }
  return TRUE;
}

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {
  // Send output Cfg msg  
    build(NODEID_RF_SCANNER, gConfig.subID, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);
    // Common payload
    sndMsg.payload.data[0] = SCANNER_GETDEV_CONFIG;
    sndMsg.payload.data[1] = offset;
    uint8_t custom_playload = 2;
    if(isNeedUniqueid != 0) 
    {
      memcpy(sndMsg.payload.data + 2,_uniqueID, UNIQUE_ID_LEN);
      custom_playload += UNIQUE_ID_LEN;
    }  
    memcpy(sndMsg.payload.data + custom_playload, (void *)((uint16_t)(&gConfig) + offset), size);
    moSetLength(size+custom_playload);
    moSetPayloadType(P_CUSTOM);
    bMsgReady = 1;
    SendMyMessage();
}
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
        if(_isAck)
        { // request nodeid response
          gConfig.nodeID = lv_nodeID;
          memcpy(gConfig.NetworkID, rcvMsg.payload.data, sizeof(gConfig.NetworkID));
          GotNodeID();
          // Increase brightness to indicate ID required
          if( gConfig.cntRFReset < MAX_RF_RESET_TIME ) {
            SetDeviceBrightness(DEFAULT_BRIGHTNESS + 10, RING_ID_ALL);
            Msg_DevBrightness(_sender);
            return 1;
          }
        }
        else
        { // change node,need register
           gConfig.nodeID = lv_nodeID;
           ResetNodeToRegister();
        }
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
      gIsConfigChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }else if( _type == I_GET_NONCE ) {
      // RF Scanner Probe
        if( _sender == NODEID_RF_SCANNER ) {
          uint8_t _lenPayl = miGetLength();
          if( rcvMsg.payload.data[0] == SCANNER_PROBE ) {      
            MsgScanner_ProbeAck();
          } else if( rcvMsg.payload.data[0] == SCANNER_SETUP_RF ) {
            if(!IS_MINE_SUBID(_sensor)) return 0;  
            Process_SetupRF(rcvMsg.payload.data + 1,_lenPayl-1);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETUPDEV_RF ) {
            if(!isIdentityEqual(rcvMsg.payload.data + 1,_uniqueID,UNIQUE_ID_LEN)) return 0;
            Process_SetupRF(rcvMsg.payload.data + 1 + UNIQUE_ID_LEN,_lenPayl-1 - UNIQUE_ID_LEN);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETCONFIG ) {
            
            if(!IS_MINE_SUBID(_sensor)) return 0;          
            uint8_t cfg_len = _lenPayl - 2;
            Process_SetConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETDEV_CONFIG ) {  
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            uint8_t cfg_len = _lenPayl - 10;
            Process_SetDevConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETDEV_CONFIG ) {  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[10];
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            MsgScanner_ConfigAck(offset,cfgblock_len,TRUE); 
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETCONFIG ) { 
            if(!IS_MINE_SUBID(_sensor)) return 0;  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[2];
            MsgScanner_ConfigAck(offset,cfgblock_len,FALSE);
          }
          return 1;
        }      
      }
    break;
    
  case C_PRESENTATION:
    if( _sensor == S_LIGHT ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        GotPresented();
        gIsStatusChanged = TRUE;
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
        uint8_t _lenPayl = miGetLength();
        if( _lenPayl == 1)
        { 
          // set main lamp(ID:1) power(V_STATUS:2) on/off
          bool _OnOff = (rcvMsg.payload.bValue == DEVICE_SW_TOGGLE ? DEVST_OnOff == DEVICE_SW_OFF : rcvMsg.payload.bValue == DEVICE_SW_ON);
          SetDeviceOnOff(_OnOff, RING_ID_ALL);
          if( _needAck ) {
            bDelaySend = (rcvMsg.header.destination == BROADCAST_ADDRESS);
            Msg_DevBrightness(_sender);
            return 1;
          }
        }
        else if( _lenPayl == 3)
        {
          uint8_t _OnOff = (rcvMsg.payload.data[0] == DEVICE_SW_TOGGLE ? DEVST_OnOff == DEVICE_SW_OFF : rcvMsg.payload.bValue == DEVICE_SW_ON);
          uint8_t unit = rcvMsg.payload.data[1];
          uint8_t time = rcvMsg.payload.data[2];
          if(unit == MINUTE_UNIT)
          {
            offdelaytick = (int32_t)time * 60 * 100; //10ms timer
          }
          else if(unit == HOUR_UNIT)
          {
            offdelaytick = (int32_t)time * 60 * 60 * 100;
          }
          else
          {
            offdelaytick = (int32_t)time * 100;
          }
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
              if(_sender == NODEID_MIN_REMOTE && rcvMsg.header.destination == BROADCAST_ADDRESS)
              { // remote broadcast msg for power test
                if( _Brightness%10 !=0 && _Brightness/10 >= 3 ) _Brightness = 50;
              }

            break;
          case OPERATOR_SUB:
            if(DEVST_Bright > rcvMsg.payload.data[1] + BR_MIN_VALUE) {
              _Brightness = DEVST_Bright - rcvMsg.payload.data[1];
            } else {
              if(_sender == NODEID_MIN_REMOTE && rcvMsg.header.destination == BROADCAST_ADDRESS)
              { // remote broadcast msg for power test
                _Brightness = 5;
              }
              else
              {
                _Brightness = BR_MIN_VALUE;
              }
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
            gIsStatusChanged = TRUE;
          }
        } else if( IS_RAINBOW(gConfig.type) || IS_MIRAGE(gConfig.type) ) {
          // Set RGBW
          if( _OnOff != RINGST_OnOff(r_index) || _Brightness != RINGST_Bright(r_index) || rcvMsg.payload.data[3] != RINGST_W(r_index) 
             || rcvMsg.payload.data[4] != RINGST_R(r_index) || rcvMsg.payload.data[5] != RINGST_G(r_index) || rcvMsg.payload.data[6] != RINGST_B(r_index) ) {
               SetDeviceHue(_OnOff, _Brightness, rcvMsg.payload.data[3], rcvMsg.payload.data[4], rcvMsg.payload.data[5], rcvMsg.payload.data[6], _RingID);
               gIsStatusChanged = TRUE;
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

//----------------------------------------------
// RF Scanner Messages
//----------------------------------------------
// Probe ack message
void MsgScanner_ProbeAck() {
  uint8_t payl_len = UNIQUE_ID_LEN + 1;
  build(NODEID_RF_SCANNER, 0x00, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);

  // Common payload
  sndMsg.payload.data[0] = SCANNER_PROBE;
  memcpy(sndMsg.payload.data + 1, _uniqueID, UNIQUE_ID_LEN);
  
  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.nodeID;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.rfChannel;
  sndMsg.payload.data[payl_len++] = (gConfig.rfDataRate << 2) + gConfig.rfPowerLevel;
  memcpy(sndMsg.payload.data + payl_len, gConfig.NetworkID, sizeof(gConfig.NetworkID));
  payl_len += sizeof(gConfig.NetworkID);
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;
//    uint8_t uniqueid[8];
//    UC ConfigBlock[15];
//}MyMsgPayload_t
#define CFGBLOCK_SIZE    15
#define CFGBLOCK_NO_UNIQUEID_SIZE CFGBLOCK_SIZE+UNIQUE_ID_LEN
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid) {
  uint8_t cfg_end_offset = cfglen;
  if(cfglen == 0) cfg_end_offset = sizeof(Config_t)-1;
  else
  {
    cfg_end_offset = offset + cfglen > sizeof(Config_t)-1?sizeof(Config_t)-1:offset + cfglen;
  }  
  while( offset < cfg_end_offset )
  {
    uint8_t left_len = cfg_end_offset - offset;
    uint8_t payl_len = left_len < CFGBLOCK_SIZE ? left_len : CFGBLOCK_SIZE;
    if(_isByUniqueid) AddCfgOutputBuf(offset,payl_len,1);
    else  
    {
      payl_len = left_len < CFGBLOCK_NO_UNIQUEID_SIZE ? left_len : CFGBLOCK_NO_UNIQUEID_SIZE;
      AddCfgOutputBuf(offset,payl_len,0);
    }
    offset+=payl_len;
    offset %= sizeof(Config_t);
  }
}

//////set config by nodeid&subid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;  //config offset
//    UC ConfigBlock[23];
//}MyMsgPayload_t
//////set config by nodeid&subid data struct/////////////////////
void Process_SetConfig(u8 _len) {
  uint8_t offset = rcvMsg.payload.data[1];
  memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2,_len);
  gIsConfigChanged = TRUE;
}
//////set config by uniqueid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;   //config offset
//    uint8_t uniqueid[8];
//    
//    UC ConfigBlock[15];
//}MyMsgPayload_t
//////set config by uniqueid data struct/////////////////////
void Process_SetDevConfig(u8 _len) {
    uint8_t offset = rcvMsg.payload.data[1];
    memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2+UNIQUE_ID_LEN,_len);
    gIsConfigChanged = TRUE;
}
bool IsNodeidValid(uint8_t nodeid)
{
  return !(IS_NOT_DEVICE_NODEID(nodeid) && !IS_GROUP_NODEID(nodeid));
}
//////set rf /////////////////////////////////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t uniqueid[8];
//    uint8_t channel;
//    uint8_t datarate;
//    uint8_t powerlevel;
//    uint8_t network[6];
//    uint8_t nodeid;        //unnecessary data field£¬has this field£¬need change nodeid£¬0 indicate ignore this parameter
//    uint8_t subid;         //unnecessary data field£¬has this field£¬need change subid
//}MyMsgPayload_t
//////set rf /////////////////////////////////////////////////
void Process_SetupRF(const UC *rfData,uint8_t rflen)
{
  bool bNeedChangeCfg = FALSE;
  if(rflen > 0 &&(*rfData)>=0 && (*rfData)<=127)
  {
    if(gConfig.rfChannel != (*rfData))
    {
      gConfig.rfChannel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 1 &&(*rfData)>=RF24_1MBPS && (*rfData)<= RF24_250KBPS)
  {
    if(gConfig.rfDataRate != (*rfData))
    {
      gConfig.rfDataRate = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 2 &&(*rfData)>=RF24_PA_MIN && (*rfData)<= RF24_PA_ERROR)
  {
    if(gConfig.rfPowerLevel != (*rfData))
    {
      gConfig.rfPowerLevel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  bool bValidNet = FALSE;
  bool newNetwork[6] = {0};
  if(rflen > 8)
  {  
    for(uint8_t i = 0;i<6;i++)
    {
      if(*(rfData+i) != 0)
      {
        bValidNet=TRUE;
        break;
      }
    }
    if(isIdentityEqual(rfData,gConfig.NetworkID,sizeof(gConfig.NetworkID)))
    {
      bValidNet=FALSE;
    }
    else
    {
      memcpy(newNetwork,rfData,sizeof(newNetwork));
    }
  }
  rfData = rfData + sizeof(gConfig.NetworkID);
  bool bNeedResetNode = FALSE;
  if(rflen > 9 && (* rfData) != 0)
  {
    if(gConfig.nodeID != (* rfData))
    {
      if(IsNodeidValid(*rfData))
      {
        gConfig.nodeID = (* rfData);
        bNeedResetNode = TRUE;
      }    
    }
  }
  rfData++; 
  if(rflen > 10)
  {
    if(gConfig.subID != (* rfData ))
    {
      gConfig.subID = (*rfData);
      bNeedChangeCfg = TRUE;
    }
  }
  if(bValidNet)
  {// nodeid is valid,allow change networkid
    if(IsNodeidValid(gConfig.nodeID))
    {
      memcpy(gConfig.NetworkID,newNetwork,sizeof(gConfig.NetworkID));
      bNeedResetNode = TRUE;
    }
  }
  if(bNeedResetNode)
    gResetNode = TRUE;
  if(gResetNode || gResetRF || bNeedChangeCfg)
  {
    gIsConfigChanged = TRUE;
  }
}
//----------------------------------------------