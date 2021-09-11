#include "KraftKontrol/modules/hid_modules/input_modules/joystick.h"



void Joystick::init() {

    //analogReadResolution(ANALOG_READ_RES);

    //pinMode(JOYSTICK_BUTTON_PIN, INPUT);
    //pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

    lpfPosX_ = LowPassFilter<float>(lpfFactor_);
    lpfPosY_ = LowPassFilter<float>(lpfFactor_);

} 


void Joystick::thread() {

    //Save last joystick state
    JoystickData lastJoystickData = joystickData_;

    //Read current state
    float xBuf;// = (float)(analogRead(JOYSTICK_X_PIN)-(1<<ANALOG_READ_RES)/2)*3.3f/(1<<ANALOG_READ_RES)/2;
    float yBuf;// = (float)(analogRead(JOYSTICK_Y_PIN)-(1<<ANALOG_READ_RES)/2)*3.3f/(1<<ANALOG_READ_RES)/2;

    bool buttonBuf;// = !digitalRead(JOYSTICK_BUTTON_PIN);

    //Update filters, state container and publish current state
    joystickData_.posX = xBuf;
    joystickData_.posY = yBuf;
    joystickData_.buttonPressed = buttonBuf;
    joystickDataTopic_.publish(joystickData_);

    //Button event detection
    if (!lastJoystickData.buttonPressed && joystickData_.buttonPressed) {
        eJoystickEvent_t event = eJoystickEvent_t::BUTTON_TRIG;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.buttonPressed && !joystickData_.buttonPressed) {
        eJoystickEvent_t event = eJoystickEvent_t::BUTTON_RETURN;
        joystickEventTopic_.publish(event);
    }

    //Left right event detection
    if (lastJoystickData.posX < joystickEventThreshold_ && joystickData_.posX >= joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::RIGHT_TRIG;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posX >= joystickEventThreshold_ && joystickData_.posX < joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::RIGHT_RETURN;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posX > -joystickEventThreshold_ && joystickData_.posX <= -joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::LEFT_TRIG;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posX <= -joystickEventThreshold_ && joystickData_.posX > -joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::LEFT_RETURN;
        joystickEventTopic_.publish(event);
    }

    //Up down event detection
    if (lastJoystickData.posY < joystickEventThreshold_ && joystickData_.posY >= joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::UP_TRIG;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posY >= joystickEventThreshold_ && joystickData_.posY < joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::UP_RETURN;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posY > -joystickEventThreshold_ && joystickData_.posY <= -joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::DOWN_TRIG;
        joystickEventTopic_.publish(event);
    } else if (lastJoystickData.posY <= -joystickEventThreshold_ && joystickData_.posY > -joystickEventThreshold_) {
        eJoystickEvent_t event = eJoystickEvent_t::DOWN_RETURN;
        joystickEventTopic_.publish(event);
    }
    
} 


