#ifndef __COLOR_FACTORY_H
#define __COLOR_FACTORY_H

#define CF_COLOR_COUNT          9

void cf_initStep();
void cf_nextStep();
uint8_t cf_getStepWhiteVal();
uint8_t cf_getStepRedVal();
uint8_t cf_getStepGreenVal();
uint8_t cf_getStepBlueVal();
uint32_t cf_getInitialColorVal();
uint32_t cf_getTargetColorVal();
void cf_updateInitialColor(uint32_t _color);
void cf_updateTargetColor(uint32_t _color);
uint32_t cf_makeColorValue(uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b);
uint32_t cf_fadeColor(uint32_t from_value, uint32_t to_value, uint8_t _delta);

#endif // __COLOR_FACTORY_H