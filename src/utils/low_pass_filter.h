#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H



#include "Arduino.h"



template<typename T>
class LowPassFilter {
public:


    /**
     * Constructor for no filtering.
     * Cutoff = 1Hz
     *
     * @param values none
     * @return none.
     */
    LowPassFilter() {

        _sampleRate = -1;
        _cutOffFreq = 1;
        _RC = 1.0/(_cutOffFreq*2*3.14);

        if (_sampleRate != -1) {
            float dt = 1.0/_sampleRate;
            _alpha = dt/(_RC+dt);
        }

    }


    /**
     * Give sampleRate to improve performance
     *
     * @param values cutOffFreq and sampleRate
     * @return none.
     */
    LowPassFilter(float cutOffFreq, int32_t sampleRate = -1) {

        _sampleRate = sampleRate;
        _cutOffFreq = cutOffFreq;
        _RC = 1.0/(_cutOffFreq*2*3.14);

        if (_sampleRate != -1) {
            float dt = 1.0/_sampleRate;
            _alpha = dt/(_RC+dt);
        }

    }


    /**
     *
     * @param values input value
     * @return filtered value.
     */
    T update(T input) {
        
        if (_sampleRate == -1) {
            float dt = (micros() - _lastRun)/1000000.0;
            _lastRun = micros();
            _alpha = dt/(_RC+dt);
        }

        T output = _lastValue + (input - _lastValue)*_alpha;
        _lastValue = output;

        return output;

    }


    /**
     * Timestamp of input value (in microseconds) to improve accuracy.
     * This overrides the sampling input at constructor.
     *
     * @param values input value and timestamp in microseconds
     * @return filtered value.
     */
    T update(T input, uint32_t timestampUS) {
        
        float dt = (timestampUS - _lastRun)/1000000.0;
        _lastRun = timestampUS;
        _alpha = dt/(_RC+dt);

        T output = _lastValue + (input - _lastValue)*_alpha;
        _lastValue = output;

        return output;

    }


    /**
     * Returns that current filtered value.
     *
     * @param values none
     * @return filtered value.
     */
    T getValue() {return _lastValue;}


    /**
     * Sets the value to given parameter
     *
     * @param values input value
     * @return none.
     */
    void setValue(const T &input) {
        
        _lastValue = input;

    }



private:

    int32_t _sampleRate;
    float _cutOffFreq;
    float _RC;

    float _alpha;

    T _lastValue;
    uint32_t _lastRun = 0;


};



#endif