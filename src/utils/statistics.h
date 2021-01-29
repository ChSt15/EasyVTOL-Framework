#ifndef STATISTICS_HELPER_H
#define STATISTICS_HELPER_H



#include "definitions.h"

#include "CircularBuffer.h"


template<typename T, size_t S> // <sample type, sample size>
class Statistics {
public:

    void addSample(T newSample) {
        _data.unshift(newSample);
    }

    bool updateValues() {

        if (_data.size() == 0) return false;

        _average = 0;
        for (uint32_t n = 0; n < _data.size(); n++) {
            _average += _data[n];
        }
        _average /= _data.size();

        if (_data.size() <= 1) return false;

        _deviation = 0;
        for (uint32_t n = 0; n < _data.size(); n++) {
            T x = _data[n] - _average;
            __deviation += x*x;
        }
        _deviation = sqrt(_deviation/(_data.size() - 1));

        return true;

    }

    T getAverage() {return _average;}
    T getDeviation() {return _deviation;}


private:

    CircularBuffer <T, S> _data;

    T _average;
    T _deviation;


};











#endif