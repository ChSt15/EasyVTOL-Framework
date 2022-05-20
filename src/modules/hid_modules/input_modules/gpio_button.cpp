#include "KraftKontrol/modules/hid_modules/input_modules/gpio_button.h"


GPIOButton::GPIOButton(GPIO_HAL_Abstract& pin, bool onLow, bool useInternalPullup, uint32_t rate): ButtonHID_Abstract(500*MILLISECONDS, rate), pin_(pin) {
    usePullup_ = useInternalPullup;
    onLow_ = onLow;
}


bool GPIOButton::getButtonValue() {

    return valueBuf_.getMedian() == 10;

}


bool GPIOButton::getRawButton() {

    return pin_.getPinValue() != onLow_;

}


void GPIOButton::forceStatusUpdate() {

    valueBuf_.clear();
    for (uint32_t i = 0; valueBuf_.availableSpace() > 0; i++) {
        valueBuf_.placeFront(getRawButton() ? 10:0, true);
    }

}


void GPIOButton::init() {

    pin_.setPinMode(eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT);
    pin_.setPinPull(usePullup_ ? eGPIO_PULLMODE_t::eGPIO_PULLMODE_PULLUP : eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE);

    forceStatusUpdate();

}


bool GPIOButton::getButtonStatus() {

    valueBuf_.placeFront(getRawButton() ? 10:0, true);
    int val = valueBuf_.getMedian();

    bool returnVal = false;
    if (val == 10) returnVal = true;
    else returnVal = false;

    return returnVal;

}
