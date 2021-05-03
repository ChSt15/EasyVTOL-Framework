#include "rgb_led.h"

/*
namespace RGBLED {

    const int ledPin = RGBLED_PIN;

    IntervalControl ledInterval(1); //Keep rate low for starting
    IntervalControl rateCalcInterval(1); 

    CRGB led;

    DeviceStatus ledStatus = DeviceStatus::DEVICE_NOT_STARTED; 
    uint8_t startAttempts = 0;

    uint32_t rate = 0;
    uint32_t loopCounter = 0;

    byte hue = 0;

    CHSV setting = CHSV(70, 255, 255);

}


void RGBLED::deviceThread() {

    if (!ledInterval.isTimeToRun()) return; 

    loopCounter++;


    if (ledStatus == DeviceStatus::DEVICE_RUNNING) {

        led = setting;
        FastLED.show();

    } else if (ledStatus == DeviceStatus::DEVICE_NOT_STARTED || ledStatus == DeviceStatus::DEVICE_RESTARTATTEMPT) {

        FastLED.addLeds<WS2812B, ledPin, RGB>(&led, 1);
        FastLED.setBrightness(255);

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


void RGBLED::setHSV(CHSV hsv) {
    setting = hsv;
}


uint32_t RGBLED::getRate() {
    return rate;
}


DeviceStatus RGBLED::getDeviceStatus() {return ledStatus;}

*/
