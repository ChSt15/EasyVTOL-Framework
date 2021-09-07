#include "KraftKontrol/modules/hid_modules/input_modules/generic_button.h"


GenericButton::GenericButton(uint32_t pin, bool useInternalPullup, uint32_t rate): ButtonHID_Abstract(500*MILLISECONDS, rate) {
    pin_ = pin;
    usePullup_ = useInternalPullup;
}


bool GenericButton::getButtonValue() {
    return valueBuf_.getMedian() == 10;
}


void GenericButton::forceStatusUpdate() {

    valueBuf_.clear();
    for (uint32_t i = 0; valueBuf_.availableSpace() > 0; i++) {
        valueBuf_.placeFront(digitalRead(pin_)?10:0, true);
    }

}


void GenericButton::init() {

    pinMode(pin_, usePullup_ ? INPUT_PULLUP:INPUT);

    for (uint32_t i = 0; valueBuf_.availableSpace() > 0; i++) {
        valueBuf_.placeFront(digitalRead(pin_)?10:0, true);
    }

}


bool GenericButton::getButtonStatus() {

    valueBuf_.placeFront(digitalRead(pin_)?10:0, true);
    int val = valueBuf_.getMedian();

    bool returnVal = false;
    if (val == 10) returnVal = true;
    else returnVal = false;

    return returnVal;

}
