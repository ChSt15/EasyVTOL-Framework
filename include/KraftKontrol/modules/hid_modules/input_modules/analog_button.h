#ifndef ANALOG_BUTTON_H
#define ANALOG_BUTTON_H



#include "KraftKontrol/modules/hid_modules/input_modules/button_abstract.h"
#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/utils/low_pass_filter.h"
#include "KraftKontrol/platforms/platform_hal.h"

#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"



class AnalogButton: public ButtonHID_Abstract {
private:

    ///Buffer for storing button values.
    LowPassFilter<float> lpf_;

    float threshold_ = 0;

    bool invert_ = false;

    Simple_Subscriber<DataTimestamped<float>> adcChannelSubr_;


public:

    /**
     * @param adcChannel Which adc channel to use as input. Should output a value between -1 and 1.
     * @param threshold Over what value should be interpreted as "pressed". Defaults to 0.5.
     * @param invert ADC channel value should be inverted. Usefull to have 2 "buttons" for each side of the axis. Defaults to false.
     * @param rate What rate this should run at. Defaults to 50Hz.
     */
    AnalogButton(const ADCChannel& adcChannel, float threshold = 0.5, bool invert = false, uint32_t rate = 50);

    /**
     * @returns button value.
     */
    bool getButtonValue();

    /**
     * @returns analog value.
     */
    float getAnalogValue();

    /**
     * Makes button take current status. Basically bypasses filter.
     */
    void forceStatusUpdate();


private:

    ///Initialises button.
    void init() override;

    /**
     * Determines button status.
     * @returns true if button pressed.
     */
    bool getButtonStatus() override;


};



#endif