#ifndef SERVO_PPM_H
#define SERVO_PPM_H



#include "Arduino.h"

#include "definitions.h"

#include "utils/device_status.h"



#define MAX_NUM_CHANNELS 20


#define STANDARD_LOW 1000
#define STANDARD_HIGH 2000

#define ONESHOT125_LOW 125
#define ONESHOT125_HIGH 250

#define ONESHOT42_LOW 42
#define ONESHOT42_HIGH 82

#define MULTISHOT_LOW 5
#define MULTISHOT_HIGH 25



class PPMChannel {
public:

    PPMChannel(int16_t pin = -1);
    ~PPMChannel();

    void deviceThread();

    DeviceStatus getDeviceStatus();

    bool setChannel(float percent);
    float getChannel();

    bool setHighTimeMaxMicros(uint32_t maxTimeUS);
    void setHighTimeMinMicros(uint32_t minTimeUS);

    uint32_t getHighTimeMaxMicros();
    uint32_t getHighTimeMinMicros();

    //bool startSending();      needs implementing. Should Start the pwm output. Other functions must be changes too.
    //void stopSending();       needs implementing. Should stop the pwm output. Other functions must be changes too.

    bool setPin(int16_t pin);

private:

    static int16_t usedPins[MAX_NUM_CHANNELS];
    static byte devices;

    int16_t _pin = -1;  

    DeviceStatus _servoStatus = DeviceStatus::DEVICE_NOT_STARTED;

    float _percent = 0.5f;

    uint32_t _highMaxUS = 2000;
    uint32_t _highMinUS = 1000;

    float _deltaHighUS = _highMaxUS - _highMinUS;
    float _periodUS = _highMaxUS*1.25f;

    const byte resolution = 12;
    const uint32_t bits = (uint32_t)1<<resolution;
    
};





#endif