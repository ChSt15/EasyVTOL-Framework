#ifndef ARDUINO_PWM_HAL_H
#define ARDUINO_PWM_HAL_H


#include "../../../KraftKontrol/hal/pwm_hal_abstract.h"

#include "Arduino.h"

#ifdef TEENSY

/**
 * Abstract for PWM output pins. To be implemented by subclass for certain platform.
 */
class PWM_HAL: public PWM_HAL_Abstract {
private:

    uint32_t pin_ = 0;

    float value_ = 0;

    bool isEnabled_ = false;

    uint32_t bits_ = 8;

    uint32_t frequency_ = 0;

    void setAnalogValue(float percent);


public:

    /**
     * Automatically sets pin to output with given value.
     * @param pin Which pin to control as output.
     * @param initValue Which dutycycle to give pin at startup. Values 0 to 1. Defaults to 0.
     * @param initFrequency Which frequency to give pwm at startup. Defaults to 500Hz.
     * @param enabled If the pwm should be enables on initialisation. Defaults to true.
     */
    PWM_HAL(uint32_t pin, float initValue = 0, uint32_t initFrequency = 500, bool enabled = true);

    ~PWM_HAL();

    /**
     * Initialises PWM output.
     */
    void init() override;

    /**
     * Enables or disables PWM output. Disabling will cause pin to float (High impedance).
     * @param enable Enable or disable. Defaults to true.
     */
    void enable(bool enable = true) override;

    /**
     * @returns Which pin this controls.
     */
    uint32_t getPin() override;

    /**
     * Sets pin value. From 0 to 1.
     * @param value Which value to set pin to.
     */
    void setPinValue(float value) override;

    /**
     * @returns current pin value. 0 to 1.
     */
    float getPinValue() override;

    /**
     * Sets pwm frequency. From 0 to 1.
     * @param value Which frequency [Hz] to set PWM to.
     */
    void setPinFrequency(uint32_t value) override;

    /**
     * @returns current PWM Frequency.
     */
    uint32_t getPinFrequency() override;
    
};

#endif

#endif