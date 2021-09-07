#ifndef BUTTON_ABSTRACT_H
#define BUTTON_ABSTRACT_H


#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"


enum class eButton_Event_t {
    ///Button was pressed.
    PRESSED,
    ///Button was released after short time. (Shorter than threshold)
    SHORT,
    ///Button is being held. (Longer than threshold)
    HOLD,
    ///Button was released.
    RELEASED
};


enum class eButton_State_t {
    RELEASED,
    PRESSED,
    HELD
};


class ButtonHID_Abstract: public Task_Abstract {
private:
    
    //Current button state.
    eButton_State_t buttonState_ = eButton_State_t::RELEASED;

    //Topic for button event.
    Topic<eButton_Event_t> buttonEventTopic_;

    //Time threshold for long button press.
    int64_t longPressThreshold_ = 0;

    //Timestamp for button pressed 
    int64_t pressTimestamp_ = 0;

    bool isPressed_ = false;

public:

    /**
     * @param longPressThresholdTime Threshold (in ns) where long press event will be triggered.
     * @param buttonReadRate Rate at which getButtonStatus() will be called. Defaults to 20Hz.
     */
    ButtonHID_Abstract(int64_t longPressThresholdTime, uint32_t buttonReadRate = 20);

    /**
     * @returns button event topic
     */
    Topic<eButton_Event_t>& getTopic(); 

    /**
     * @returns current button status. True if pressed.
     */
    bool isPressed() const;

    /**
     * @returns current button status.
     */
    eButton_State_t getState() const;


protected:

    /**
     * To be implemented by subclass.
     * Will be called at rate set in constructor to retrieve button status.
     * Buttons should be debounced and accurate. Algorithms for this are to be placed in this function.
     * @returns true if button pressed.
     */
    virtual bool getButtonStatus() = 0;

    /**
     * Can be overridden by subclass.
     * Is called once at after creation. Gives time to initialise button.
     */
    virtual void init() override; //ToDo: Do we really need this?


private:

    /**
     * Manages data and calls getButtonStatus().
     */
    void thread() override;


};


#endif