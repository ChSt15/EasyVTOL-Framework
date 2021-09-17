#include "KraftKontrol/modules/hid_modules/display_modules/st7735_driver.h"



ST7735Driver::ST7735Driver(SPIClass& spi, int backlightPin, int CSPin, int DCPin, int RSTPin): Task_Abstract("ST7735 Driver", 1, 0), display_(&spi, CSPin, DCPin, RSTPin), buffer_(getDisplayWidth(), getDisplayHeight()) {

    backlightPin_ = backlightPin;

    //numDisplayPixels_ = (uint32_t)getDisplayHeight()*getDisplayWidth();
    //displayBuffer_ = new uint16_t[numDisplayPixels_];

}

ST7735Driver::~ST7735Driver() {}


void ST7735Driver::clear() {
    buffer_.fillScreen(display_.color565(0,0,0));
}

void ST7735Driver::update() {

    display_.startWrite();
    display_.setAddrWindow(0,0, getDisplayWidth(), getDisplayHeight());
    display_.writePixels(buffer_.getBuffer(), getDisplayWidth()*getDisplayHeight());
    display_.endWrite();

}

uint16_t ST7735Driver::getDisplayWidth() const {
    return 160;
}

uint16_t ST7735Driver::getDisplayHeight() const {
    return 128;
}

void ST7735Driver::drawPixel(uint16_t x, uint16_t y, const Color& color) {
    buffer_.drawPixel(x, y, display_.color565(color.red, color.green, color.blue));
}

void ST7735Driver::drawText(uint16_t x, uint16_t y, const Color& color, uint16_t size, const char* text) {
    buffer_.setCursor(x, y);
    buffer_.setTextSize(size);
    buffer_.print(text);
}


void ST7735Driver::init() {

    pinMode(backlightPin_, OUTPUT);
    digitalWrite(backlightPin_, HIGH);

    display_.initR(INITR_BLACKTAB);
    display_.setRotation(3);

    display_.setSPISpeed(50000000);

    moduleStatus_ = eModuleStatus_t::eModuleStatus_Running;

    Serial.println("Display Start Success.");

    stopTaskThreading();

}
