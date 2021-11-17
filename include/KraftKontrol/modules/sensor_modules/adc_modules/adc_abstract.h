#ifndef ADC_ABSTRACT_H
#define ADC_ABSTRACT_H



#include "stdint.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



/**
 * Class for ADCChannel data. Takes care of topics and values.
 */
class ADCChannel {
private:

    Topic<DataTimestamped<float>> adcChannelTopic_;

    DataTimestamped<float> adcValue_ = 0;

    float offset_ = 0;
    float scaling_ = 1;


public:

    /**
     * Sets the offset for the voltage.
     * output = voltage*scaling + offset;
     * @param offset Voltage offset.
     */
    void setOffset(float offset);

    /**
     * Sets the scaling factor for the voltage.
     * output = voltage*scaling + offset;
     * @param scaling Voltage scaling factor.
     */
    void setScaling(float scaling);

    /**
     * @returns the adc channels topic.
     */
    const Topic<DataTimestamped<float>>& getADCChannelTopic() const;

    /**
     * Updates internal ADC value and publishes it to topic.
     * @param value New adc value with timestamp and publish to topic.
     */
    void setValue(const DataTimestamped<float>& value);

    /**
     * @returns adc value.
     */
    const DataTimestamped<float>& getValue() const;


};


/**
 * Abstract class for ADC modules. Needs to be implemented by subclass.
 */
class ADC_Abstract {
public:

    virtual ~ADC_Abstract();

    /**
     * @param channel Which ADC channel topic to return reference for.
     * @returns reference to adc data topic
     */
    virtual ADCChannel& operator[](uint32_t channel) = 0;

    /**
     * @param channel Which ADC channel topic to return reference for.
     * @returns reference to adc data topic
     */
    const ADCChannel& operator[](uint32_t channel) const; 

    /**
     * @returns number of channels ADC has
     */
    virtual uint32_t getNumberChannels() const = 0;

    
};





#endif