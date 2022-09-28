#ifndef GPIO_BUTTON_H
#define GPIO_BUTTON_H



#include "KraftKontrol/modules/hid_modules/input_modules/button_abstract.h"
#include "KraftKontrol/utils/buffer.h"
#include "KraftKontrol/platforms/platform_hal.h"

#include "KraftKontrol/hal/gpio_hal_abstract.h"



class GPIOButton: public ButtonHID_Abstract {
private:

    ///Buffer for storing button values.
    Buffer<int, 3> valueBuf_;
    uint32_t numberReadings_;

    bool usePullup_ = false;

    bool onLow_ = true;

    GPIO_HAL_Abstract& pin_;


public:

    /**
     * @param pin Which pin to use as input.
     * @param onLow Button is pressed when pin is measured as low. Defaults to true.
     * @param useInternalPullup Use internal pullup. Defaults to true;
     * @param rate What rate to read button at.
     * @param numberReadings What number of readings to save in buffer for averaging. Max 20.
     */
    GPIOButton(GPIO_HAL_Abstract& pin, bool onLow = true, bool useInternalPullup = true, uint32_t rate = 50);

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

    bool getRawButton();


};



#endif