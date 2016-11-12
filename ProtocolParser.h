#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_Presentation();
void Msg_DevOnOff(uint8_t _to, uint8_t _dest);
void Msg_DevBrightness(uint8_t _to, uint8_t _dest);
void Msg_DevCCT(uint8_t _to, uint8_t _dest);
void Msg_DevStatus(uint8_t _to, uint8_t _dest, uint8_t _ring);
void Msg_DevTopology(uint8_t _to, uint8_t _dest, uint8_t _ring);

#endif /* __PROTOCOL_PARSER_H */