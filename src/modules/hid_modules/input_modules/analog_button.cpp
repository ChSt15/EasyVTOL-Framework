#include "KraftKontrol/modules/hid_modules/input_modules/analog_button.h"


AnalogButton::AnalogButton(const ADCChannel& adcChannel, float threshold, bool invert, uint32_t rate): ButtonHID_Abstract(500*MILLISECONDS, rate) {

    adcChannelSubr_.subscribe(adcChannel.getADCChannelTopic());
    invert_ = invert;
    threshold_ = threshold;

    lpf_ = 20;

}


bool AnalogButton::getButtonValue() {

    float value = lpf_.getValue();

    if (invert_) value = -value;

    return (value > threshold_ ? true : false);

}


float AnalogButton::getAnalogValue() {

    return lpf_.getValue();

}


void AnalogButton::forceStatusUpdate() {

    lpf_.setValue(adcChannelSubr_.getItem().sensorData);

}


void AnalogButton::init() {}


bool AnalogButton::getButtonStatus() {

    lpf_.update(adcChannelSubr_.getItem().sensorData);

    return getButtonValue();

}
