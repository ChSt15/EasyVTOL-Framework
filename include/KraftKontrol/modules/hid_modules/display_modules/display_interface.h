#ifndef DISPLAY_INTERFACE_H
#define DISPLAY_INTERFACE_H



#include "Arduino.h"



class Display_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Draws a string on the display at a coordinate. Not giving coordinates will make it draw under last line at left most.
     *
     * @param string Test to write
     * @param x X-Coordinate 
     * @param y Y-Coordinate 
     */
    virtual void drawString(String string, int16_t x = -1, int16_t y = -1) = 0;

    /**
     * Clears everything on the display
     *
     */
    virtual void clearDisplay() = 0;

    /**
     * Updates display with new changes.
     */
    virtual void updateDisplay() = 0;

    /**
     * Returns true if pressure data available
     *
     * @param greyscaleBitmap Pointer to array of bytes containing greyscale of each pixel
     * @param x X-Coordinate 
     * @param y Y-Coordinate 
     * @param width Width of bitmap in pixels
     * @param height Height of bitmap in pixels 
     */
    virtual void drawBitmap(uint8_t* greyscaleBitmap, int16_t x, int16_t y, int16_t width, int16_t height) = 0;



private:



    
};





#endif