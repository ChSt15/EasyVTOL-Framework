#ifndef ARDUINO_PIN_INTERRUPT_HAL_H
#define ARDUINO_PIN_INTERRUPT_HAL_H


#include "Arduino.h"

#include "KraftKontrol/utils/list.h"


class PinInterrupt_HAL {
public:

    /**
     * @param pin Pin to trigger interrupt.
     * @param function Function to run on trigger.
     * @param onRise When true then interrupt will be called on rising edge, else on falling. Defaults to true.
     * @param enable If true then interrupt will immediatly be enabled. Disable if this is unwanted. Defaults to true.
     */
    PinInterrupt_HAL(uint32_t pin, void (*function)(void), bool onRise = true, bool enable = true) {
        if (function == nullptr) return;
        pin_ = pin;
        function_ = function;
        onRise_ = onRise;
        reAttachPin();
    }

    /**
     * Destructor, Interrupt must be removed.
     */
    ~PinInterrupt_HAL() {
        detachInterrupt(pin_);
    }

    /**
     * Changes the pin to trigger interrupt.
     * @param pin Pin to trigger interrupt.
     */
    void setPin(uint32_t pin) {
        pin_ = pin;
        reAttachPin();
    }

    /**
     * @returns pin currently used to trigger interrupt.
     */
    uint32_t getPin() {return pin_;}

    /**
     * Changes the function to be called on interrupt.
     * @param function Function to run on trigger.
     */
    void setFunction(void (*function)(void)) {
        function_ = function;
        reAttachPin();
    }

    /**
     * @returns pointer to function that is run on interrupt.
     */
    void (*getFunction())(void) {return function_;} //I should have used a typedef :(

    /**
     * Can stop or start the interrupt from getting triggered.
     * @param enable True to enable interrupt, false to disable.
     */
    void setEnable(bool enable) {
        if (enable) {
            reAttachPin();
        } else {
            detachInterrupt(pin_);
        }
    }


private:

    void reAttachPin() {
        detachInterrupt(pin_);
        attachInterrupt(pin_, function_, onRise_ ? RISING:FALLING);
    }

    uint32_t pin_ = 0;

    void (*function_)(void) = nullptr;

    bool onRise_ = true;
    
};





#endif 
