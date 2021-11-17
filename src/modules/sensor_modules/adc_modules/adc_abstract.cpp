#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"


void ADCChannel::setOffset(float offset) {
    offset_ = offset;
}


void ADCChannel::setScaling(float scaling) {
    scaling_ = scaling;
}


const Topic<DataTimestamped<float>>& ADCChannel::getADCChannelTopic() const {
    return adcChannelTopic_;
}


void ADCChannel::setValue(const DataTimestamped<float>& value) {
    adcValue_.data = value.data*scaling_ + offset_;
    adcValue_.timestamp = value.timestamp;
    adcChannelTopic_.publish(adcValue_);
}


const DataTimestamped<float>& ADCChannel::getValue() const {
    return adcValue_;
}



ADC_Abstract::~ADC_Abstract() {}

const ADCChannel& ADC_Abstract::operator[](uint32_t channel) const {
    return (*this)[channel];
}

