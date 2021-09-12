#include "KraftKontrol/modules/hid_modules/display_modules/display_abstract.h"




void Display_Abstract::drawBoxFilled(uint16_t x, uint16_t y, const Color& color, uint16_t w, uint16_t h) {};

void Display_Abstract::drawBox(uint16_t x, uint16_t y, const Color& color, uint16_t w, uint16_t h, uint16_t wallWidth) {};

void Display_Abstract::drawLine(uint16_t x, uint16_t y, const Color& color, uint16_t endX, uint16_t endY, uint16_t lineWidth) {};

void Display_Abstract::drawCircleFilled(uint16_t x, uint16_t y, const Color& color, uint16_t radius) {};

void Display_Abstract::drawCircle(uint16_t x, uint16_t y, const Color& color, uint16_t endX, uint16_t endY, uint16_t lineWidth) {};

void Display_Abstract::drawTriangleFilled(uint16_t x, uint16_t y, const Color& color, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3) {};

void Display_Abstract::drawTriangle(uint16_t x, uint16_t y, const Color& color, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t lineWidth) {};

void Display_Abstract::enableDisplay(bool enable) {};

uint32_t Display_Abstract::getBufferIndex(uint16_t x, uint16_t y) {
    uint32_t index = (uint32_t) x*getDisplayHeight() + y%getDisplayWidth();
    index = index%(getDisplayHeight()*getDisplayWidth()); //Make sure it stays within bounds.
    return index;
}

