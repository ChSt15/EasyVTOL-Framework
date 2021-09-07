#ifndef ARDUINO_GPIO_HAL_H
#define ARDUINO_GPIO_HAL_H


#include "Arduino.h"

#include "../../../KraftKontrol/hal/gpio_hal_abstract.h"


/**
 * Arduino implementation for GPIO_HAL.
 * Controls a pin
 */
class GPIO_HAL: public GPIO_HAL_Abstract {
private:

    //Which pin this GPIO is.
    uint32_t pin_ = 0;

    //Current pin IO mode.
    eGPIO_IOMODE_t pinMode_ = eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT;

    //Current pin pull mode.
    eGPIO_PULLMODE_t pinPullMode_ = eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE;


public:


    /**
     * @param pin Which pin to control.
     * @param pinIOMode Set pin at input or output. Defaults to input without pullup or pulldown.
     */
    GPIO_HAL(uint32_t pin, eGPIO_IOMODE_t pinIOMode = eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT);

    /**
     * Automatically sets pin to output with given value.
     * @param pin Which pin to control as output.
     * @param initValue Which value to give pin at startup.
     */
    GPIO_HAL(uint32_t pin, bool initValue);

    /**
     * Automatically sets pin to input with given pullmode.
     * @param pin Which pin to control.
     * @param pullMode Give pin pullup, pulldown or no pull.
     */
    GPIO_HAL(uint32_t pin, eGPIO_PULLMODE_t pullMode);

    /**
     * Places pin into input state without pullmode.
     */
    ~GPIO_HAL();

    /**
     * @returns Which pin this controls.
     */
    uint32_t getPin() override;

    /**
     * Sets pin value.
     * @param value Which value to set pin to.
     */
    void setPinValue(bool value) override;

    /**
     * @returns current pin value.
     */
    bool getPinValue() override;

    /**
     * Sets pin to given mode. Input/output.
     * @param mode Which mode to set pin to.
     */
    void setPinMode(eGPIO_IOMODE_t mode) override;

    /**
     * @returns current pin mode.
     */
    eGPIO_IOMODE_t getPinMode() override;

    /**
     * Sets pin to use given pullup or pulldown resistor or to remove.
     * @param mode Which pull mode to set pin to.
     */
    void setPinPull(eGPIO_PULLMODE_t pull) override;

    /**
     * @returns current pin pullmode.
     */
    eGPIO_PULLMODE_t getPinPull() override;

    

};




#endif 
