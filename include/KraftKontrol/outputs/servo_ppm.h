#ifndef SERVO_PPM_H
#define SERVO_PPM_H



#include "Arduino.h"



#ifndef DEGREES
    #define DEGREES (3.14159265f/180.0f)
#endif



/**
 * Used to transfer protocol information.
 * 
 * To add a new protocol add the name to to this and add the protocol 
 * timings to the switch statement in _setProtocolTiming() function.
 */
enum ePPMProtocol_t {
    ePPMProtocol_Standard_1000us,
    ePPMProtocol_Oneshot_125us,
    ePPMProtocol_Oneshot_42us,
    ePPMProtocol_Multishot_5us
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

    PPMChannel(int16_t pin, ePPMProtocol_t protocol, float offset = 0, float scaler = 1);
    ~PPMChannel();

    bool setAngle(float angle, bool limit = true);

    bool setChannel(float percent, bool limit = true);
    float getChannel();

    void activateChannel(bool activate = true);
    bool getActive();

    void setProtocol(const ePPMProtocol_t &protocol);

    bool setPin(int16_t pin);

private:

    void _getProtocolTiming(uint32_t &min, uint32_t &max, ePPMProtocol_t protocol);

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