#include "rgb_led.h"


namespace RGBLED {

    const int ledPin = 4;

    IntervalControl ledInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    CRGB led;

    DeviceStatus ledStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    byte hue = 0;

}


void RGBLED::deviceThread() {

    if (!ledInterval.isTimeToRun()) return; 

    loopCounter++;


    if (ledStatus == DeviceStatus::DEVICE_RUNNING) {

        led.setHSV(hue++, 255, 255);

    } else if (ledStatus == DeviceStatus::DEVICE_NOT_STARTED || ledStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        FastLED.addLeds<WS2812B, ledPin, RGB>(&led, 1);

        ledInterval.setRate(LED_FPS);

        ledStatus = DeviceStatus::DEVICE_RUNNING;

    } else { //This section is for device failure or a wierd mode that should not be set, therefore assume failure

        ledStatus = DeviceStatus::DEVICE_FAILURE;
        ledInterval.block(true);
        rate = 0;

    }



    if (rateCalcInterval.isTimeToRun()) {
        rate = loopCounter;
        loopCounter = 0;
    }

}


uint32_t RGBLED::getRate() {
    return rate;
}


DeviceStatus RGBLED::getDeviceStatus() {return ledStatus;}


