#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H



#include "stdint.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/sensor_timestamp.h"





class ADCChannel_Interface {
public:

    /**
     * @returns reference to adc data topic
     */
    Topic<SensorTimestamp<float>>& getADCTopic() {return adcTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<SensorTimestamp<float>> adcTopic_;

    
};





#endif