#include "servo_ppm.h"



PPMChannel::PPMChannel(int16_t pin, const ePPMProtocol_t &protocol, float offset, float scaler) {
    _pin = pin;
    _offset = offset;
    _scaler = scaler;
    setProtocol(protocol);
    pinMode(_pin, OUTPUT);
    #ifndef ESP32 
        analogWriteResolution(_resolution);
    #endif
    setChannel(-1);
}


PPMChannel::~PPMChannel() {

    if (_pin == -1) return;
    #ifndef ESP32 
        analogWrite(_pin, 0);
    #endif
    digitalWrite(_pin, LOW);

}


/**
    * Sets the channels value between -100% and 100%
    * Returns if the set was a success (Fail if pin was not set)
    *
    * @param values float percent
    * @return bool.
*/
bool PPMChannel::setChannel(const float &percent, const bool &limit) {

    if (_pin == -1) return false;
    
    _percent = percent*_scaler + _offset; //apply offsets and scalers
    _percent = (_percent+1)/2; //Remap to 0-1 
    if (limit) _percent = constrain(_percent, 0.0f, 1.0f); //constrain

    float dutyCycle = ((float)_highMinUS + _percent*_deltaHighUS)/_periodUS;

    dutyCycle = constrain(dutyCycle, 0.0f, 1.0f);

    #ifndef ESP32 
    if (_active) analogWrite(_pin, dutyCycle*_maxValue);
    else {
        analogWrite(_pin, 0);
        digitalWrite(_pin, LOW);
    }
    #endif

    return true;

}


/**
    * Sets the servo to a certain angle
    *
    * @param values angle, offset and scaler
    * @return bool.
*/
bool PPMChannel::setAngle(const float &angle, const bool &limit) {

    float output = angle/(45.0f*DEGREES);

    return setChannel(output, limit);

}


/**
    * Turns channel off or on.
    *
    * @param values angle, offset and scaler
    * @return bool.
*/
void PPMChannel::activateChannel(const bool &activate) {

    if (activate == _active) return;

    _active = activate;
    setChannel(_percent);

}


/**
    * Returns if channel is active
    *
    * @param values angle, offset and scaler
    * @return bool.
*/
bool PPMChannel::getActive() {

    return _active;

}


/**
    * Gets the channels value between 0% and 100%
    * Later this can be changed to simulate servo movements
    *
    * @param values none
    * @return float.
*/
float PPMChannel::getChannel() {return _percent;}


/**
    * Sets the given parameters to the min max of the timing.
    *
    * @param values min, max, protocol
    * @return none.
*/
void PPMChannel::_getProtocolTiming(uint32_t &min, uint32_t max, const ePPMProtocol_t &protocol) {
    
    switch (protocol) {
    case ePPMProtocol_t::ePPMProtocol_Standard_1000us :
        min = 1000;
        max = 2000;
        break;

    case ePPMProtocol_t::ePPMProtocol_Oneshot_125us :
        min = 125;
        max = 250;
        break;

    case ePPMProtocol_t::ePPMProtocol_Oneshot_42us :
        min = 42;
        max = 82;
        break;

    case ePPMProtocol_t::ePPMProtocol_Multishot_5us :
        min = 5;
        max = 25;
        break;
    
    default: //Default will set timing to standard 1000us
        min = 1000;
        max = 2000;
        break;
    }

}


/**
    * Sets the PPMProtocol timing according to the 
    * given protocol.
    *
    * @param values protocol
    * @return none.
*/
void PPMChannel::setProtocol(const ePPMProtocol_t &protocol) {

    if (_pin == -1) return; //Make sure a pin has been selected

    _getProtocolTiming(_highMinUS, _highMaxUS, protocol); //Get the timing from protocol type

    _highMaxUS = max(_highMaxUS, _highMinUS); //Limit max to min
    uint32_t frequencyHZ = 1000000/(_highMaxUS*1.25f);
    _periodUS = 1000000/frequencyHZ;
    _deltaHighUS = _highMaxUS - _highMinUS;

    #ifndef ESP32 
        analogWriteFrequency(_pin, frequencyHZ); //Update frequency
    #endif

}


/**
    * Changes the Pin used for the PPMChannel.
    * Returns if was success.
    *
    * @param values int16_t pin.
    * @return bool.
*/
bool PPMChannel::setPin(int16_t pin) {

    if (pin == -1) return false;

    digitalWrite(_pin, LOW); //remove old pin from channel
    pinMode(_pin, INPUT);

    _pin = pin; //change pins

    pinMode(_pin, OUTPUT); //add new pin to channel

    #ifndef ESP32 
        analogWriteResolution(_resolution);
        analogWriteFrequency(_pin, 1000000/_periodUS);
    #endif
    
    setChannel(_percent); //Update channel

    return true;

}


