#ifndef IBUS_RX_H
#define IBUS_RX_H

#include "Arduino.h"

#include "definitions.h"

#include "CircularBuffer.h"

#include "interval_control.h"
#include "utils/device_status.h"



#define COMMS_RATE 100



#define IBUS_CHANNEL_ROLL 0
#define IBUS_CHANNEL_PITCH 1
#define IBUS_CHANNEL_THROTTLE 2
#define IBUS_CHANNEL_YAW 3
#define IBUS_CHANNEL_SWA 4
#define IBUS_CHANNEL_SWB 5
#define IBUS_CHANNEL_SWC 6
#define IBUS_CHANNEL_SWD 7
#define IBUS_CHANNEL_SWE 8
#define IBUS_CHANNEL_VRA 9
#define IBUS_CHANNEL_VRB 10
#define IBUS_CHANNEL_AUX1 11
#define IBUS_CHANNEL_AUX2 12
#define IBUS_CHANNEL_AUX3 13


/**
 * Struct containing rx data.
 * Channel min value is -1 and max is 1.
 */
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