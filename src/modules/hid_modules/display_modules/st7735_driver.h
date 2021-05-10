#ifndef ST7735_DRIVER_H
#define ST7735_DRIVER_H

#ifdef ESP32



#include "Arduino.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "display_interface.h"

#include "modules/module_abstract.h"

#include "TFT_eSPI.h"

#include "utils/circular_buffer.h"



class ST7735Driver: public Display_Interface, public Module_Abstract, public Task_Abstract {
public:

    ST7735Driver(int backlightPin) : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_Low, true) {
        backlightPin_ = backlightPin;
    }
    
    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @return uint32_t.
     */
    uint32_t loopRate() {return loopRate_;}

    /**
     * Returns update rate (in Hz) of the display
     *
     * @return uint32_t.
     */
    uint32_t updateRate() {return updateRate_;}

    /**
     * Draws a string on the display at a coordinate. Not giving coordinates will make it draw under last line at left most.
     *
     * @param string Test to write
     * @param x X-Coordinate 
     * @param y Y-Coordinate 
     */
    void drawString(String string, int16_t x = -1, int16_t y = -1);

    /**
     * Clears everything on the display
     *
     */
    void clearDisplay();

    void updateDisplay() {}

    /**
     * Returns true if pressure data available
     *
     * @param greyscaleBitmap Pointer to array of bytes containing greyscale of each pixel
     * @param x X-Coordinate 
     * @param y Y-Coordinate 
     * @param width Width of bitmap in pixels
     * @param height Height of bitmap in pixels 
     */
    void drawBitmap(uint8_t* greyscaleBitmap, int16_t x, int16_t y, int16_t width, int16_t height);

    /**
     * Sets the backlight brightness
     *
     * @param brightness Backlight setting. 0 is off and 1 is full on
     */
    void setBacklight(bool brightness) {
        if (brightness) digitalWrite(backlightPin_, HIGH);
        else digitalWrite(backlightPin_, LOW);
    }

    /**
     * Puts display into sleep mode
     *
     * @param sleep True to put display into sleep, false to get out of sleep
     */
    void enableSleep(bool sleep = true) {
        if (sleep) display_.writecommand(ST7735_DISPOFF);
        else display_.writecommand(ST7735_DISPON);
    }


private:

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    TFT_eSPI display_;
    int backlightPin_;

    uint8_t currentPin_ = 0;

    uint8_t startAttempts_ = 0;

    uint32_t loopRate_ = 0;
    uint32_t loopCounter_ = 0;

    uint32_t updateRate_ = 0;
    uint32_t updateCounter_ = 0;

    uint32_t lastMeasurement_ = 0;

    bool block_ = false;



    
};




#endif
#endif