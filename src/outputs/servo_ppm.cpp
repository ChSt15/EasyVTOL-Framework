#include "servo_ppm.h"



byte PPMChannel::devices = 0;


PPMChannel::PPMChannel(int16_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    if (devices == 0) {
        for (byte n = 0; n < MAX_NUM_CHANNELS; n++) usedPins[n] = -1;
    }

    devices++;

    _servoStatus = DeviceStatus::DEVICE_RUNNING;
}


PPMChannel::~PPMChannel() {
    devices--;
}


/**
    * deviceThread is currently not used
    *
    * @param values none
    * @return none.
*/
void PPMChannel::deviceThread() {}


/**
    * Gives the current status of the PPMChannel
    *
    * @param values none
    * @return DeviceStatus.
*/
DeviceStatus PPMChannel::getDeviceStatus() {return _servoStatus;}


/**
    * Sets the channels value between 0% and 100%
    * Returns if the set was a success (Fail if pin was not set)
    *
    * @param values float percent
    * @return bool.
*/
bool PPMChannel::setChannel(float percent) {

    if (_pin == -1) return false;
    
    _percent = constrain(percent, 0.0f, 1.0f); 

    float dutyCycle = ((float)_highMinUS + _percent*_deltaHighUS)/_periodUS;

    analogWrite(_pin, dutyCycle*bits);

    return true;
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
    * Sets the PPMProtocol Max on time of signal
    * and then changes the frequency accordingly.
    * Returns if this was a success (Fail if pin not set)
    *
    * @param values uint32_t maxTimeUS in microseconds
    * @return bool.
*/
bool PPMChannel::setHighTimeMaxMicros(uint32_t maxTimeUS) {

    if (_pin == -1) return false; //Make sure pin has been selected

    _highMaxUS = max(maxTimeUS, _highMinUS); //Limit max to min
    uint32_t frequencyHZ = 1000000/(_highMaxUS*1.25f);
    _periodUS = 1000000/frequencyHZ;
    _deltaHighUS = _highMaxUS - _highMinUS;

    analogWriteFrequency(_pin, frequencyHZ); //Update frequency

    return true;
}


/**
    * Sets the PPMProtocol Min on time of signal.
    *
    * @param values uint32_t minTimeUS in microseconds
    * @return none.
*/
void PPMChannel::setHighTimeMinMicros(uint32_t minTimeUS) {

    _highMinUS = min(minTimeUS, _highMaxUS); //Limit min to max
    _deltaHighUS = _highMaxUS - _highMinUS; 

}


/**
    * Returns protocol Max High Time
    *
    * @param values none.
    * @return uint32_t time in microseconds.
*/
uint32_t PPMChannel::getHighTimeMaxMicros() {return _highMaxUS;}


/**
    * Returns protocol Min High Time
    *
    * @param values none.
    * @return uint32_t time in microseconds.
*/
uint32_t PPMChannel::getHighTimeMinMicros() {return _highMinUS;}


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
    digitalWrite(_pin, LOW);

    analogWriteFrequency(_pin, 1000000/_periodUS);
    
    setChannel(_percent); //Update channel

    return true;

}


