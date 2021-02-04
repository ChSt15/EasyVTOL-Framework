#ifndef IBUS_RX_H
#define IBUS_RX_H

#include "Arduino.h"

#include "definitions.h"

#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"



#define COMMS_RATE 100



namespace IBUSReceiver {

    extern CircularBuffer <float[16], 100> channelData;

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif