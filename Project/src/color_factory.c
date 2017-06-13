#include <stm8s.h>
#include "color_factory.h"

// Color arrays
uint8_t cf_lstColor[CF_COLOR_COUNT][4] = { { 0, 255, 128, 0 }, // orange
                                           { 0, 255, 255, 0 }, // yellow
                                           { 0, 205, 0, 205 }, // Magenta
                                           { 0, 0, 255, 0 },   // green
                                           { 0, 0, 255, 255 }, // Cyan
                                           { 0, 0, 0, 255 },   // blue
                                           { 0, 0, 191, 255 }, // DeepSkyBlue
                                           { 0, 227, 31, 51 }, // purple
                                           { 0, 255, 0, 0 } }; // red

// Initialize color variables
uint8_t cf_step = 0;
uint32_t cf_initialColor = 0;
uint32_t cf_targetColor = 0;

uint32_t cf_makeColorValue(uint8_t _w, uint8_t _r, uint8_t _g, uint8_t _b) {
  uint32_t _color = _w;
  _color <<= 8;
  _color += _r;
  _color <<= 8;
  _color += _g;
  _color <<= 8;
  _color += _b;  
  return _color;
}

uint8_t cf_getStepWhiteVal() {
  return cf_lstColor[cf_step][0];
}

uint8_t cf_getStepRedVal() {
  return cf_lstColor[cf_step][1];
}

uint8_t cf_getStepGreenVal() {
  return cf_lstColor[cf_step][2];
}

uint8_t cf_getStepBlueVal() {
  return cf_lstColor[cf_step][3];
}

uint32_t cf_getStepColorVal(uint8_t _step) {
  return cf_makeColorValue(cf_lstColor[_step][0], cf_lstColor[_step][1], cf_lstColor[_step][2], cf_lstColor[_step][3]);
}

uint32_t cf_getInitialColorVal() {
  return cf_initialColor;
}

uint32_t cf_getTargetColorVal() {
  return cf_targetColor;
}

void cf_refreshStepColor() {
  uint8_t pre_step = (cf_step > 0 ? cf_step - 1 : CF_COLOR_COUNT - 1);
  
  cf_initialColor = cf_getStepColorVal(pre_step);
  cf_targetColor = cf_getStepColorVal(cf_step);
}

void cf_updateInitialColor(uint32_t _color) {
  cf_initialColor = _color;
}

void cf_updateTargetColor(uint32_t _color) {
  cf_targetColor = _color;
}

void cf_initStep() {
  cf_step = 0;
  cf_refreshStepColor();
}

void cf_nextStep() {
  cf_step++;
  cf_step %= CF_COLOR_COUNT;
  cf_refreshStepColor();
}

uint32_t cf_fadeColor(uint32_t from_value, uint32_t to_value, uint8_t _delta) {
  uint32_t new_value = 0;
  uint8_t color1, color2;

  for( uint8_t i = 0; i < 4; i++ ) {
    color1 = (from_value & 0xFF);
    color2 = (to_value & 0xFF);
    if( color1 > color2 + _delta) {
      color1 -= _delta;
    } else if( color2 > color1 + _delta ) {
      color1 += _delta;
    } else {
      color1 = color2;
    }
    uint32_t temp = color1;
    temp <<= (i * 8);
    new_value += temp;
    from_value >>= 8;
    to_value >>= 8;
  }
  
  return new_value;    
}