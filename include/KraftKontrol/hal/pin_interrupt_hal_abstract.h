#ifndef PIN_INTERRUPT_HAL_ABSTRACT_H
#define PIN_INTERRUPT_HAL_ABSTRACT_H


#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"
#include "KraftKontrol/utils/list.h"

#include "gpio_hal_abstract.h"



enum ePinInterruptMode_t : uint32_t {
    ///Triggered on rising edge
    ePinInterruptMode_Rising,
    ///Triggered on falling edge
    ePinInterruptMode_Falling,
    ///Triggered on high
    ePinInterruptMode_High,
    ///Triggered on low
    ePinInterruptMode_Low,
    ///Triggered on change
    ePinInterruptMode_Change
};


template <typename TYPE>
class PinInterrupt_HAL {
private:

    //Which pin is connected.
    GPIO_HAL_Abstract& pin_;

    //Pin interrupt mode. Rising, Falling etc.
    ePinInterruptMode_t mode_;

    //Task to be resumed on interrupt. Is a nullptr when no task is to be resumed
    Task_Threading* taskToResume = nullptr;

    //If interrupt triggered then this is set to true until pin was reset.
    bool triggered_ = false;

    bool enabled_ = false;

    //Function to call on interrupt trigger
    void (TYPE::*callbackFunction_)(void) = nullptr;

    //List of attached interrupts
    static List<PinInterrupt_HAL*>& interruptList();


public:

    /**
     * @param pin Pin to trigger interrupt.
     * @param function Function to run on interrupt.
     * @param interruptMode When to call interrupt, on rising, falling etc.
     * @param enable If true then interrupt will immediatly be enabled. Disable if this is unwanted. Defaults to true.
     * @param pullMode If internal pullup or pulldown should be connected to pin.
     */
    PinInterrupt_HAL(GPIO_HAL_Abstract& gpioPin, void (TYPE::*function)(void), ePinInterruptMode_t interruptMode, bool enable = true, eGPIO_PULLMODE_t pullMode = eGPIO_PULLMODE_t::eGPIO_PULLMODE_NONE): pin_(gpioPin),  {
        mode_ = interruptMode;
        callbackFunction_ = function;
        pin_.setPinMode(eGPIO_IOMODE_t::eGPIO_IOMODE_INPUT);
        pin_.setPinPull(pullMode);
        enabled_ = enable;
        if (enable) attachPin();

    }

    /**
     * Destructor, Interrupt must be removed.
     */
    virtual ~PinInterrupt_HAL() {
        detachPin();
    }

    /**
     * Sets what task will be resumed when interrupt occured.
     * @param task Which task to resume.
     */
    void setTaskToResume(Task_Threading& task) {
        taskToResume = &task;
    }

    /**
     * Removes the task which was being resumed on interrupts.
     */
    void removeTaskToResume() {
        taskToResume = nullptr;
    }

    /**
     * @returns pointer to task that gets resumed in interrupt. Returns nullptr when no task is being resumed.
     */
    const Task_Threading* getTaskToResume() {
        return taskToResume;
    }

    /**
     * Changes the pin to trigger interrupt.
     * @param pin Pin to trigger interrupt.
     */
    void setPin(GPIO_HAL_Abstract& pin) {
        pin_ = pin;
        attachPin();
    }

    /**
     * @returns pin currently used to trigger interrupt.
     */
    const GPIO_HAL_Abstract& getPin() {return pin_;}

    /**
     * Changes the function to be called on interrupt.
     * @param function Function to run on trigger.
     */
    void setFunction(void (TYPE::*function)(void)) {
        callbackFunction_ = function;
    }

    /**
     * @returns pointer to function that is run on interrupt.
     */
    void (TYPE::*getFunction())(void) {return callbackFunction_;}

    /**
     * Can stop or start the interrupt from getting triggered.
     * @param enable True to enable interrupt, false to disable.
     */
    void enable(bool enable = true) {
        enabled_ = enable;
        if (enable) {
            attachPin();
        } else {
            detachPin(pin_);
        }
    }


protected:

    /**
     * Needs to be implemented by subclass (platform dependant).
     * Attaches globalInterruptHandler() to pin_. Trigger should be on pin change.
     */
    virtual void attachPin() = 0;

    /**
     * Needs to be implemented by subclass (platform dependant).
     * Detaches globalInterruptHandler() from pin_.
     */
    virtual void detachPin() = 0;

    /**
     * Needs to be attached to interrupt pin via attachPin.
     * Searches for pin that that caused interrupt and calls callback Function.
     */
    static void globalInterruptHandler();

    
};


template<typename TYPE>
List<PinInterrupt_HAL<TYPE>*>& PinInterrupt_HAL<TYPE>::interruptList() {

    static List<PinInterrupt_HAL<TYPE>*> interruptList_;
    return interruptList;

}




#endif 
