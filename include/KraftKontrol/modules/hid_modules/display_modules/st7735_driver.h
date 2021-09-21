#ifndef ST7735_DRIVER_H
#define ST7735_DRIVER_H



#include "Arduino.h"

#include "display_abstract.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"
#include "KraftKontrol/modules/module_abstract.h"

#include "lib/Adafruit-ST7735-Library-master/Adafruit_ST7735.h"



class ST7735Driver: public Display_Abstract, public Task_Abstract, public Module_Abstract {
public:

    ST7735Driver(SPIClass& spi, int backlightPin, int CSPin, int DCPin, int RSTPin = -1);

    ~ST7735Driver() override;


    Adafruit_ST7735& getDisplay() {return display_;}

    void init() override;

    /**
     * Clears everything on the display
     *
     */
    void clear() override;

    /**
     * Updates display with new changes.
     */
    void update() override;

    /**
     * @returns screen width in pixels.
     */
    uint16_t getDisplayWidth() const override;

    /**
     * @returns screen height in pixels.
     */
    uint16_t getDisplayHeight() const override;

    /**
     * Draws a single pixel on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     */
    void drawPixel(uint16_t x, uint16_t y, const Color& color) override;

    /**
     * Draws a string on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param size Text size.
     * @param text Text to write
     */
    void drawText(uint16_t x, uint16_t y, const Color& color, uint16_t size, const char* text) override;


private:

    Adafruit_ST7735 display_;
    int backlightPin_;

    uint8_t startAttempts_ = 0;

    GFXcanvas16 buffer_;

    
};



#endif