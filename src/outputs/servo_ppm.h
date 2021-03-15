#ifndef SERVO_PPM_H
#define SERVO_PPM_H



#include "Arduino.h"

#include "definitions.h"

#include "utils/device_status.h"



/**
 * Used to transfer protocol information.
 * 
 * To add a new protocol add the name to to this and add the protocol 
 * timings to the switch statement in _setProtocolTiming() function.
 */
enum PPM_PROTOCOL {
    STANDARD_1000,
    ONESHOT_125,
    ONESHOT_42,
    MULTISHOT
};


/**
 * Class used to create PPM channels on pins to control 
 * Servos and ESCs.
 * Due to this using PWM modules, if the CPU were to lock-up 
 * then the PWM signal may continue to run the ESC/Servo at 
 * the last given command. Meaning Motors may not turn off!
 */
class PPMChannel {
public:

    PPMChannel(int16_t pin = -1);
    ~PPMChannel();

    void deviceThread();

    DeviceStatus getDeviceStatus();

    bool setChannel(const float &percent);
    float getChannel();

    void setProtocol(const PPM_PROTOCOL &protocol);

    //bool startSending();      needs implementing. Should Start the pwm output. Other functions must be changes too.
    //void stopSending();       needs implementing. Should stop the pwm output. Other functions must be changes too.

    bool setPin(int16_t pin);

private:

    void _getProtocolTiming(uint32_t &min, uint32_t max, const PPM_PROTOCOL &protoco);

    int16_t _pin = -1;  

    DeviceStatus _servoStatus = DeviceStatus::DEVICE_NOT_STARTED;

    float _percent = 0.5f;

    uint32_t _highMaxUS = 2000;
    uint32_t _highMinUS = 1000;

    float _deltaHighUS = _highMaxUS - _highMinUS;
    float _periodUS = _highMaxUS*1.25f;

    static const byte resolution = 12;                      //static const because these should not change
    static const uint32_t bits = (uint32_t)1<<resolution;
    
};





#endif