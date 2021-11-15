#ifndef GPIO_HAL_ABSTRACT_H
#define GPIO_HAL_ABSTRACT_H


#include "stdint.h"


/**
 * Sets pin to be either input of output.
 */
enum class eGPIO_IOMODE_t {
    eGPIO_IOMODE_INPUT,
    eGPIO_IOMODE_OUTPUT
};

/**
 * Sets pins pullup/pulldown resistor.
 */
enum class eGPIO_PULLMODE_t {
    //Pullup resistor
    eGPIO_PULLMODE_PULLUP,
    //Pulldown resistor
    eGPIO_PULLMODE_PULLDOWN,
    //No pullup or pulldown.
    eGPIO_PULLMODE_NONE
};


/**
 * Abstract for GPIO pins. To be implemented by subclass for certain platform.
 */
class GPIO_HAL_Abstract {
public:

    virtual ~GPIO_HAL_Abstract() {} //Do nothing

    /**
     * Initialises GPIO.
     */
    virtual void init() = 0;

    /**
     * @returns Which pin this controls.
     */
    virtual uint32_t getPin() = 0;

    /**
     * Sets pin value.
     * @param value Which value to set pin to.
     */
    virtual void setPinValue(bool value) = 0;

    /**
     * @returns current pin value.
     */
    virtual bool getPinValue() = 0;

    /**
     * Sets pin to given mode. Input/output.
     * @param mode Which mode to set pin to.
     */
    virtual void setPinMode(eGPIO_IOMODE_t mode) = 0;

    /**
     * @returns current pin mode.
     */
    virtual eGPIO_IOMODE_t getPinMode() = 0;

    /**
     * Sets pin to use given pullup or pulldown resistor or to remove.
     * @param mode Which pull mode to set pin to.
     */
    virtual void setPinPull(eGPIO_PULLMODE_t pull) = 0;

    /**
     * @returns current pin pullmode.
     */
    virtual eGPIO_PULLMODE_t getPinPull() = 0;
    

};


#endif