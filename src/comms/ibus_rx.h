#ifndef IBUS_RX_H
#define IBUS_RX_H

#include "Arduino.h"

#include "definitions.h"

#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"



#define COMMS_RATE 100


struct RCChannelData {
    float channelData[16];
    bool failsafe = true;
};


namespace IBUSReceiver {

    extern CircularBuffer <RCChannelData, 100> channelDataFifo;

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif