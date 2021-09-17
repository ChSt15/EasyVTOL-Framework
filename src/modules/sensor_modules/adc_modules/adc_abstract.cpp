#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"


void ADCChannel::setOffset(float offset) {
    offset_ = offset;
}


void ADCChannel::setScaling(float scaling) {
    scaling_ = scaling;
}


const Topic<SensorTimestamp<float>>& ADCChannel::getADCChannelTopic() const {
    return adcChannelTopic_;
}


void ADCChannel::setValue(const SensorTimestamp<float>& value) {
    adcValue_.sensorData = value.sensorData*scaling_ + offset_;
    adcValue_.sensorTimestamp = value.sensorTimestamp;
    adcChannelTopic_.publish(adcValue_);
}


const SensorTimestamp<float>& ADCChannel::getValue() const {
    return adcValue_;
}



ADC_Abstract::~ADC_Abstract() {}

const ADCChannel& ADC_Abstract::operator[](uint32_t channel) const {
    return (*this)[channel];
}

