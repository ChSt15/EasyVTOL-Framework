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

    PPMChannel(int16_t pin, const PPM_PROTOCOL &protocol, float offset = 0, float scaler = 1);
    ~PPMChannel();

    bool setAngle(const float &angle, const bool &limit = true);

    bool setChannel(const float &percent, const bool &limit = true);
    float getChannel();

    void activateChannel(const bool &activate = true);
    bool getActive();

    void setProtocol(const PPM_PROTOCOL &protocol);

    bool setPin(int16_t pin);

private:

    void _getProtocolTiming(uint32_t &min, uint32_t max, const PPM_PROTOCOL &protocol);

    int16_t _pin = -1;  

    bool _active = false;

    float _offset = 0;
    float _scaler = 1;

    float _percent = 0.5f;

    uint32_t _highMaxUS = 2000;
    uint32_t _highMinUS = 1000;

    float _deltaHighUS = _highMaxUS - _highMinUS;
    float _periodUS = _highMaxUS*1.25f;

    static const byte _resolution = 12;                      //static const because these should not change
    static const uint32_t _maxValue = (uint32_t)1<<_resolution;
    
};





#endif