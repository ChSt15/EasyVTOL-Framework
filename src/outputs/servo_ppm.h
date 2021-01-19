#ifndef SERVO_PPM_H
#define SERVO_PPM_H



#include "Arduino.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"



#define STANDARD_LOW 1000
#define STANDARD_HIGH 2000



class PPMChannel {
public:

    enum PROTOCOL {
        PROTOCOL_STANDARD,
        PROTOCOL_ONESHOT125,
        PROTOCOL_MULTISHOT
    };

    static void deviceThread();

    DeviceStatus getDeviceStatus();

    void setPos();

    float getPos();

private:

    static IntervalControl servoInterval;

    byte _protocol = PROTOCOL_STANDARD;

    DeviceStatus servoStatus = DeviceStatus::DEVICE_NOT_STARTED;
    
};





#endif