#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"





class ADCChannel_Interface {
public:

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t loopRate() = 0;

    /**
     * Returns rate (in Hz) of new sensor data
     *
     * @param channel which channel to check the rate
     */
    virtual uint32_t measurementRate() = 0;

    /**
     * @returns reference to adc data topic
     */
    Topic<SensorTimestamp<float>>& getADCTopic() {return adcTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<float>> adcTopic_;

    
};





#endif