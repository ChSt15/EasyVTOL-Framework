#include "KraftKontrol/modules/hid_modules/input_modules/button_abstract.h"



ButtonHID_Abstract::ButtonHID_Abstract(int64_t longPressThresholdTime, uint32_t buttonReadRate): Task_Abstract("Button Abstract", buttonReadRate, eTaskPriority_t::eTaskPriority_Middle) {
    longPressThreshold_ = longPressThresholdTime;
}


Topic<eButton_Event_t>& ButtonHID_Abstract::getButtonTopic() {
    return buttonEventTopic_;
}


bool ButtonHID_Abstract::isPressed() const {
    return isPressed_;
}


eButton_State_t ButtonHID_Abstract::getButtonState() const {
    return buttonState_;
}


void ButtonHID_Abstract::init() {} //Do nothing if not overridden.


void ButtonHID_Abstract::thread() {

    isPressed_ = getButtonStatus();

    switch (buttonState_) {
    case eButton_State_t::RELEASED :
        if (isPressed_) {
            pressTimestamp_ = NOW();
            buttonState_ = eButton_State_t::PRESSED;
            buttonEventTopic_.publish(eButton_Event_t::PRESSED);
        }
        break;

    case eButton_State_t::PRESSED :
        if (NOW() - pressTimestamp_ >= longPressThreshold_) {
            buttonState_ = eButton_State_t::HELD;
            buttonEventTopic_.publish(eButton_Event_t::HOLD);
        } else if (!isPressed_) {
            buttonState_ = eButton_State_t::RELEASED;
            buttonEventTopic_.publish(eButton_Event_t::SHORT);
        }
        break;

    case eButton_State_t::HELD :
        if (!isPressed_) {
            buttonState_ = eButton_State_t::RELEASED;
            buttonEventTopic_.publish(eButton_Event_t::RELEASED);
        }
        break;
    
    default:
        buttonState_ = eButton_State_t::RELEASED;
        break;
    }

}

