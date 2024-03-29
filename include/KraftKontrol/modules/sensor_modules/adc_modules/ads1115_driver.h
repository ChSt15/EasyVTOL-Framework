#ifndef ADS1115_DRIVER_H
#define ADS1115_DRIVER_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "adc_abstract.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/ADS1X15-master/ADS1X15.h"

#include "KraftKontrol/utils/buffer.h"



class ADS1115Driver: public ADC_Abstract, public Module_Abstract, public Task_Threading {
public:

    ADS1115Driver(TwoWire& i2cBus, uint32_t rate = 100);
    
    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * @param channel Which ADC channel topic to return reference for.
     * @returns reference to adc data topic
     */
    virtual ADCChannel& operator[](uint32_t channel) override;

    /**
     * @returns number of channels ADC has. For this device 4.
     */
    virtual uint32_t getNumberChannels() const override {return 4;}


private:

    void _getData();


    ADCChannel adcChannels_[4];

    int chipSelectPin_ = 0;
    TwoWire& i2cBus_;
    ADS1115 adc_;

    uint8_t currentPin_ = 0;

    uint8_t startAttempts_ = 0;

    uint32_t lastMeasurement_ = 0;

    
};





#endif