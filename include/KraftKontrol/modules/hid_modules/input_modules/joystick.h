#ifndef JOYSTICK_H
#define JOYSTICK_H


#include "KraftKontrol/gui/menu_abstract.h"

#include "KraftKontrol/utils/topic_subscribers.h"

#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"
#include "KraftKontrol/modules/hid_modules/input_modules/gpio_button.h"
#include "KraftKontrol/modules/hid_modules/input_modules/analog_button.h"

#include "KraftKontrol/utils/low_pass_filter.h"



//Contains Joystick position and botton data.
struct JoystickData {
public:

    float posX = 0;
    float posY = 0;

    bool buttonPressed = false;

};


class Joystick: public Task_Abstract, public MenuControl_Abstract {
private:

    Topic<JoystickData> joystickDataTopic_;

    JoystickData joystickData_;

    const float lpfFactor_ = 10; 

    const float joystickThreshold_ = 0.3f;

    AnalogButton xAxisPositive_;
    AnalogButton xAxisNegative_;
    Simple_Subscriber<eButton_Event_t> xAxisPositiveSubr_;
    Simple_Subscriber<eButton_Event_t> xAxisNegativeSubr_;

    AnalogButton yAxisPositive_;
    AnalogButton yAxisNegative_;
    Simple_Subscriber<eButton_Event_t> yAxisPositiveSubr_;
    Simple_Subscriber<eButton_Event_t> yAxisNegativeSubr_;

    GPIOButton button_;
    Simple_Subscriber<eButton_Event_t> buttonSubr_;


public:

    /**
     * ADC channels should output a value between -1 and 1.
     * No button given. This will never output a signal for button1.
     * @param adcXChannel ADC channel for x axis.
     * @param adcYChannel ADC channel for y axis.
     */
    Joystick(const ADCChannel& adcXChannel, const ADCChannel& adcYChannel);

    /**
     * ADC channels should output a value between -1 and 1.
     * @param adcXChannel ADC channel for x axis.
     * @param adcYChannel ADC channel for y axis.
     * @param buttonPin GPIO pin connected to joystick button.
     */
    Joystick(const ADCChannel& adcXChannel, const ADCChannel& adcYChannel, int buttonPin);

    void init() override;

    void thread() override;

    /**
     * @returns raw joystick data.
     */
    const JoystickData& getJoystickData() const {return joystickData_;}

    /**
     * @returns topic for raw joystick data.
     */
    Topic<JoystickData>& getJoystickDataTopic() {return joystickDataTopic_;};


};



#endif