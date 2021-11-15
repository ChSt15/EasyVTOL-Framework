#include "KraftKontrol/modules/output_modules/hbridge/hbridge_bts7960_driver.h"



HBridge_BTS7960_Driver::HBridge_BTS7960_Driver(PWM_HAL_Abstract& positivePWMPin, PWM_HAL_Abstract& negativePWMPin, uint32_t frequency): positivePWMPin_(positivePWMPin), negativePWMPin_(negativePWMPin) {
    frequency_ = frequency;
}

void HBridge_BTS7960_Driver::init() {

    positivePWMPin_.init();
    negativePWMPin_.init();

    positivePWMPin_.setPinFrequency(frequency_);
    negativePWMPin_.setPinFrequency(frequency_);

    positivePWMPin_.setPinValue(0);
    negativePWMPin_.setPinValue(0);

}

void HBridge_BTS7960_Driver::setOutput(float power) {

    output_ = power > 1.0f ? 1.0f : (power < -1.0f ? -1.0f : power);

    if (power > 0) {
        positivePWMPin_.setPinValue(power);
        negativePWMPin_.setPinValue(0);
    } else {
        positivePWMPin_.setPinValue(0);
        negativePWMPin_.setPinValue(-power);
    }

}

float HBridge_BTS7960_Driver::getOutput() {
    return output_;
}

