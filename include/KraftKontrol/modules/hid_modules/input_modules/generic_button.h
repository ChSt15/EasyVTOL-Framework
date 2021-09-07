#ifndef GENERIC_BUTTON_H
#define GENERIC_BUTTON_H


#include "Arduino.h"

#include "KraftKontrol/modules/hid_modules/input_modules/button_abstract.h"
#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/utils/low_pass_filter.h"



class GenericButton: public ButtonHID_Abstract {
private:

    ///Buffer for storing button values.
    Buffer<int, 9> valueBuf_;

    uint32_t pin_ = 0;

    bool usePullup_ = false;


public:

    /**
     * @param pin Which pin to use as input.
     * @param useInternalPullup Use internal pullup. Defaults to true;
     */
    GenericButton(uint32_t pin, bool useInternalPullup = true, uint32_t rate = 50);

    /**
     * @returns touch value.
     */
    bool getButtonValue();

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