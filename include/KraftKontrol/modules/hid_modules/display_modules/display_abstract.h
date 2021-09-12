#ifndef DISPLAY_ABSTRACT_H
#define DISPLAY_ABSTRACT_H



#include "stdint.h"


/**
 * Allows control of display color when drawing. Uses HSV system.
 */ 
struct Color {

    Color(uint8_t brightness) {
        this->red = brightness;
        this->green = brightness;
        this->blue = brightness;
    }

    Color(uint8_t red, uint8_t green, uint8_t blue) {
        this->red = red;
        this->green = green;
        this->blue = blue;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

};


/**
 * Allows menus to control any display type. 
 * Subclass must implement core functions for drawing. 
 * Other non core functions can be overridden. 
 */
class Display_Abstract {
public:

    //Below till MARKER, are the functions that must be implemented by subclasses.

    /**
     * Clears everything on the display
     *
     */
    virtual void clear() = 0;

    /**
     * Updates display with new changes.
     */
    virtual void update() = 0;

    /**
     * @returns screen width in pixels.
     */
    virtual uint16_t getDisplayWidth() const = 0;

    /**
     * @returns screen height in pixels.
     */
    virtual uint16_t getDisplayHeight() const = 0;

    /**
     * Draws a single pixel on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     */
    virtual void drawPixel(uint16_t x, uint16_t y, const Color& color) = 0;

    /**
     * Draws a string on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param size Text size.
     * @param text Text to write
     */
    virtual void drawText(uint16_t x, uint16_t y, const Color& color, uint16_t size, const char* text) = 0;




    //################################### MARKER #######################################


    virtual ~Display_Abstract() {}


    /**
     * Draws a filled square on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param w Width of square.
     * @param h Height of square.
     */
    virtual void drawBoxFilled(uint16_t x, uint16_t y, const Color& color, uint16_t w, uint16_t h);

    /**
     * Draws an empty square on the display at a coordinate.
     *
     * @param x X-Coordinate.
     * @param y Y-Coordinate.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param w Width of square.
     * @param h Height of square.
     * @param wallWidth Width of box walls in pixels. Defaults to 1.
     */
    virtual void drawBox(uint16_t x, uint16_t y, const Color& color, uint16_t w, uint16_t h, uint16_t wallWidth = 1);

    /**
     * Draws a line on the display at given coordinates.
     *
     * @param x X-Coordinate of line start.
     * @param y Y-Coordinate of line start.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param endX X-Coordinate of line end.
     * @param endY Y-Coordinate of line end.
     * @param lineWidth Width of line in pixels. Defaults to 1.
     */
    virtual void drawLine(uint16_t x, uint16_t y, const Color& color, uint16_t endX, uint16_t endY, uint16_t lineWidth = 1);

    /**
     * Draws a filled circle on the display at a coordinate.
     *
     * @param x X-Coordinate of circle center.
     * @param y Y-Coordinate of circle center.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param radius Radius of circle in pixels.
     */
    virtual void drawCircleFilled(uint16_t x, uint16_t y, const Color& color, uint16_t radius);

    /**
     * Draws a filled circle on the display at a coordinate.
     *
     * @param x X-Coordinate of circle center.
     * @param y Y-Coordinate of circle center.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param radius Radius of circle in pixels.
     * @param lineWidth Width of circle walls in pixels. Defaults to 1.
     */
    virtual void drawCircle(uint16_t x, uint16_t y, const Color& color, uint16_t endX, uint16_t endY, uint16_t lineWidth = 1);

    /**
     * Draws a filled triangle on the display at a coordinate.
     *
     * @param x X-Coordinate point 1.
     * @param y Y-Coordinate point 1.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param x2 X-Coordinate point 2.
     * @param y2 Y-Coordinate point 2.
     * @param x3 X-Coordinate point 3.
     * @param y3 Y-Coordinate point 3.
     */
    virtual void drawTriangleFilled(uint16_t x, uint16_t y, const Color& color, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);

    /**
     * Draws a triangle on the display at a coordinate.
     *
     * @param x X-Coordinate point 1.
     * @param y Y-Coordinate point 1.
     * @param color Which color to Draw with. Monochrome displays will only draw white.
     * @param x2 X-Coordinate point 2.
     * @param y2 Y-Coordinate point 2.
     * @param x3 X-Coordinate point 3.
     * @param y3 Y-Coordinate point 3.
     * @param lineWidth Width of triangle walls in pixels. Defaults to 1.
     */
    virtual void drawTriangle(uint16_t x, uint16_t y, const Color& color, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t lineWidth = 1);

    /**
     * Will turn display on or off. 
     * Display state may be lost when turning display off!
     * @param enable True to turn display on, False to turn off. Defaults to true.
     */
    virtual void enableDisplay(bool enable = true);


protected:  

    //Helper functions.

    /**
     * @returns which index in buffer array x and y coordinates corrisponds to.
     */
    uint32_t getBufferIndex(uint16_t x, uint16_t y);

    
};





#endif