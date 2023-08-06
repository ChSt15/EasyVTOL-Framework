#include "../../../include/KraftKontrol/platforms/arduino_platform/arduino_gpio_hal.h"



GPIO_HAL::GPIO_HAL(uint32_t pin, eGPIO_IOMODE_t pinIOMode) {

    pin_ = pin;

    pinMode_ = pinIOMode;
    pinPullMode_ = eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE;

    if (pinIOMode == eGPIO_IOMODE_t::eGPIO_IOMODE_OUTPUT) {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
    } else {
        pinMode(pin_, INPUT);
    }

}

GPIO_HAL::GPIO_HAL(uint32_t pin, bool initValue) {

    pin_ = pin;

    pinMode_ = eGPIO_IOMODE_t::eGPIO_IOMODE_OUTPUT;
    pinPullMode_ = eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE;

    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, initValue);

}

GPIO_HAL::GPIO_HAL(uint32_t pin, eGPIO_PULLMODE_t pullMode) {

    pin_ = pin;

    pinMode_ = eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT;
    pinPullMode_ = pullMode;

    if (pinPullMode_ == eGPIO_PULLMODE_t::eGPIO_PULLMODE_PULLUP) {
        pinMode(pin_, INPUT);
        pinMode(pin_, INPUT_PULLUP);
    } else if (pinPullMode_ == eGPIO_PULLMODE_t::eGPIO_PULLMODE_PULLDOWN) {
        pinMode(pin_, INPUT);
        pinMode(pin_, INPUT_PULLDOWN);
    } else {
        pinMode(pin_, INPUT);
    }

}

GPIO_HAL::~GPIO_HAL() {
    pinMode(pin_, INPUT);
}

void GPIO_HAL::init() {}

uint32_t GPIO_HAL::getPin() {
    return pin_;
}

void GPIO_HAL::setPinValue(bool value) {

    if (pinMode_ != eGPIO_IOMODE_t::eGPIO_IOMODE_OUTPUT) return; //We are an input not output. Dont do anything

    digitalWrite(pin_, value);

}

bool GPIO_HAL::getPinValue() {
    return digitalRead(pin_);
}

void GPIO_HAL::setPinMode(eGPIO_IOMODE_t mode) {

    pinMode_ = mode;

    switch (pinMode_) {
    case eGPIO_IOMODE_t::eGPIO_IOMODE_OUTPUT :
        pinMode(pin_, OUTPUT);
        break;

    case eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT :
        pinMode(pin_, INPUT);
        break;
    
    default:
        pinMode(pin_, INPUT);
        break;
    }

}

eGPIO_IOMODE_t GPIO_HAL::getPinMode() {
    return pinMode_;
}

void GPIO_HAL::setPinPull(eGPIO_PULLMODE_t pull) {

    pinPullMode_ = pull;

    switch (pinPullMode_) {
    case eGPIO_PULLMODE_t::eGPIO_PULLMODE_PULLDOWN:
        pinMode(pin_, INPUT_PULLDOWN);
        break;

    case eGPIO_PULLMODE_t::eGPIO_PULLMODE_PULLUP:
        pinMode(pin_, INPUT_PULLUP);
        break;

    case eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE:
        pinMode(pin_, INPUT);
        break;
    
    default:
        pinMode(pin_, INPUT);
        break;
    }

}

eGPIO_PULLMODE_t GPIO_HAL::getPinPull() {
    return pinPullMode_;
}

