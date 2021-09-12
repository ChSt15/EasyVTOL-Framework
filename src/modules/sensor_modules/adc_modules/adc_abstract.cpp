#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"


const Topic<SensorTimestamp<float>>& ADCChannel::getADCChannelTopic() const {
    return adcChannelTopic_;
}


void ADCChannel::setValue(const SensorTimestamp<float>& value) {
    adcValue_ = value;
    adcChannelTopic_.publish(adcValue_);
}


const SensorTimestamp<float>& ADCChannel::getValue() const {
    return adcValue_;
}

ADC_Abstract::~ADC_Abstract() {}

