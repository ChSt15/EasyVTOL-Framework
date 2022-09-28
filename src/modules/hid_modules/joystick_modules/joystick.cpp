#include "KraftKontrol/modules/hid_modules/input_modules/joystick.h"




Joystick::Joystick(const ADCChannel& adcXChannel, const ADCChannel& adcYChannel, GPIO_HAL_Abstract& buttonPin):
    Task_Threading("Joystick", eTaskPriority_t::eTaskPriority_VeryHigh, SECONDS/100),
    xAxisPositive_(adcXChannel, joystickThreshold_, false),
    xAxisNegative_(adcXChannel, joystickThreshold_, true),
    yAxisPositive_(adcYChannel, joystickThreshold_, false),
    yAxisNegative_(adcYChannel, joystickThreshold_, true),
    button_(buttonPin)
{

    buttonSubr_.subscribe(button_.getButtonTopic());

    xAxisPositiveSubr_.subscribe(xAxisPositive_.getButtonTopic());
    xAxisNegativeSubr_.subscribe(xAxisNegative_.getButtonTopic());
    yAxisPositiveSubr_.subscribe(yAxisPositive_.getButtonTopic());
    yAxisNegativeSubr_.subscribe(yAxisNegative_.getButtonTopic());

    buttonSubr_.setTaskToResume(*this);
    xAxisPositiveSubr_.setTaskToResume(*this);
    xAxisNegativeSubr_.setTaskToResume(*this);
    yAxisPositiveSubr_.setTaskToResume(*this);
    yAxisNegativeSubr_.setTaskToResume(*this);
    
}


void Joystick::init() {} 


void Joystick::thread() {

    //Save last joystick state
    JoystickData lastJoystickData = joystickData_;


    //Update filters, state container and publish current state
    joystickData_.posX = xAxisPositive_.getAnalogValue();
    joystickData_.posY = yAxisPositive_.getAnalogValue();
    joystickData_.buttonPressed = button_.getButtonValue();
    joystickDataTopic_.publish(joystickData_);


    //Do button state stuff if button given.
    if (buttonSubr_.isDataNew()) {

        eButton_Event_t buttonEvent = buttonSubr_.getItem();

        switch (buttonEvent)
        {
        case eButton_Event_t::SHORT :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_Button1Short);
            break;

        case eButton_Event_t::HOLD :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_Button1Long);
            break;
        
        default:
            break;
        }

    }
    
    
    //Do analog button stuff
    if (xAxisPositiveSubr_.isDataNew()) {

        eButton_Event_t buttonEvent = xAxisPositiveSubr_.getItem();

        switch (buttonEvent)
        {
        case eButton_Event_t::SHORT :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_UpShort);
            break;

        case eButton_Event_t::HOLD :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_UpLong);
            break;
        
        default:
            break;
        }

    }

    if (xAxisNegativeSubr_.isDataNew()) {

        eButton_Event_t buttonEvent = xAxisNegativeSubr_.getItem();

        Serial.println(uint32_t(buttonEvent));

        switch (buttonEvent)
        {
        case eButton_Event_t::SHORT :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_DownShort);
            break;

        case eButton_Event_t::HOLD :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_DownLong);
            break;
        
        default:
            break;
        }

    }

    if (yAxisPositiveSubr_.isDataNew()) {

        eButton_Event_t buttonEvent = yAxisPositiveSubr_.getItem();

        switch (buttonEvent)
        {
        case eButton_Event_t::SHORT :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_RightShort);
            break;

        case eButton_Event_t::HOLD :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_RightLong);
            break;
        
        default:
            break;
        }

    }

    if (yAxisNegativeSubr_.isDataNew()) {

        eButton_Event_t buttonEvent = yAxisNegativeSubr_.getItem();

        switch (buttonEvent)
        {
        case eButton_Event_t::SHORT :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_LeftShort);
            break;

        case eButton_Event_t::HOLD :
            menuInputTopic_.publish(eMenuControlInput_t::eMenuControlInput_LeftLong);
            break;
        
        default:
            break;
        }

    }

    //stopTaskThreading();
    
} 


