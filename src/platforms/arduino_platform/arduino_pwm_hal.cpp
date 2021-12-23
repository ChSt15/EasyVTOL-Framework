#include "../../../include/KraftKontrol/platforms/arduino_platform/arduino_pwm_hal.h"

#ifdef TEENSY

PWM_HAL::PWM_HAL(uint32_t pin, float initValue, uint32_t initFrequency, bool enabled) {

    pin_ = pin;
    value_ = initValue;
    frequency_ = initFrequency;
    isEnabled_ = enabled;

}

PWM_HAL::~PWM_HAL() {

    pinMode(pin_, INPUT);

}

void PWM_HAL::init() {

    if (isEnabled_) {

        pinMode(pin_, OUTPUT);

        analogWriteFrequency(pin_, frequency_);
        setAnalogValue(value_);

    } else {

        pinMode(pin_, INPUT);

    }

}

void PWM_HAL::enable(bool enable) {

    if (enable && !isEnabled_) {
        isEnabled_ = true;

        pinMode(pin_ , OUTPUT);
        setAnalogValue(value_);

    } else if (!enable && isEnabled_) {
        isEnabled_ = false;

        pinMode(pin_, INPUT);

    }

}

uint32_t PWM_HAL::getPin() {
    return pin_;
}

void PWM_HAL::setPinValue(float value) {

    value_ = constrain(value, 0, 1);

    if (!isEnabled_) return;

    setAnalogValue(value_);

}

float PWM_HAL::getPinValue() {
    return value_;
}

void PWM_HAL::setPinFrequency(uint32_t value) {

    frequency_ = value;
    analogWriteFrequency(pin_, frequency_);

}

uint32_t PWM_HAL::getPinFrequency() {
    return frequency_;
}

void PWM_HAL::setAnalogValue(float percent) {

    percent = constrain(percent, 0.0f, 1.0f);

    uint32_t maxValue = 1<<bits_;

    analogWrite(pin_, maxValue*percent);

}

#endif
