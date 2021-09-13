#ifndef JOYSTICK_H
#define JOYSTICK_H


#include "KraftKontrol.h"



//State changes from joystick
enum class eJoystickEvent_t {
    LEFT_TRIG,
    LEFT_RETURN,
    RIGHT_TRIG,
    RIGHT_RETURN,
    UP_TRIG,
    UP_RETURN,
    DOWN_TRIG,
    DOWN_RETURN,
    BUTTON_TRIG,
    BUTTON_RETURN
};


//Contains Joystick position and botton data.
struct JoystickData {
public:

    float posX = 0;
    float posY = 0;

    bool buttonPressed = false;

};


class Joystick: public Task_Abstract {
public:

    Joystick(): Task_Abstract("Joystick", 30, eTaskPriority_t::eTaskPriority_VeryHigh) {}

    void init() override;

    void thread() override;

    const JoystickData& getJoystickData() const {return joystickData_;}

    Topic<eJoystickEvent_t>& getJoystickEventTopic() {return joystickEventTopic_;};
    Topic<JoystickData>& getJoystickDataTopic() {return joystickDataTopic_;};


private:

    Topic<eJoystickEvent_t> joystickEventTopic_;

    Topic<JoystickData> joystickDataTopic_;

    JoystickData joystickData_;

    LowPassFilter<float> lpfPosX_, lpfPosY_;

    const float lpfFactor_ = 10; 

    const float joystickEventThreshold_ = 0.7f;


};



#endif