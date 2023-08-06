#ifndef HBRIDGE_BTS7960_DRIVER
#define HBRIDGE_BTS7960_DRIVER


#include "hbridge_abstract.h"
#include "KraftKontrol/hal/gpio_hal_abstract.h"
#include "KraftKontrol/hal/pwm_hal_abstract.h"



class HBridge_BTS7960_Driver: public HBridge_Abstract {
private:

    PWM_HAL_Abstract& positivePWMPin_;
    PWM_HAL_Abstract& negativePWMPin_;

    uint32_t frequency_;

    float output_ = 0;


public:

    HBridge_BTS7960_Driver(PWM_HAL_Abstract& positivePWMPin, PWM_HAL_Abstract& negativePWMPin, uint32_t frequency = 500);

    /**
     * Initialises driver.
     */
    void init();

    /**
     * Sets the power of the output. Negative values will reverse the output.
     * @param power Output power in from -1 to 1. 
     */
    void setOutput(float power) override;

    /**
     * @returns current output power from -1 to 1.
     */
    virtual float getOutput() override;


};

#endif