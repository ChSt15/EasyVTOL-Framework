#ifndef RGB_LED_H
#define RGB_LED_H



#include "Arduino.h"

#include "FastLED.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define LED_FPS 60



namespace RGBLED {

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif