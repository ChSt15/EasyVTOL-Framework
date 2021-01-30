#ifndef STATISTICS_HELPER_H
#define STATISTICS_HELPER_H


/**
 * This class stores samples and calculates the average
 * of the stored samples and the deviation from average.
 * Usefull for filtering and sensorfusion.
*/


#include "definitions.h"

#include "CircularBuffer.h"


template<typename T, size_t S> // <sample type, sample size>
class Statistics {
public:

    /**
     * Constructor for statistics class.
     * <type, size> 
     * "Type" is the data type (uint32_t, int float etc.)
     * "size" is the maximum number of samples that will be stored.
     * 
     *
     * @param values none.
     * @return none.
     */
    Statistics();

    /**
     * Adds a new sample. Will overwrite oldest sample if full.
     * 
     *
     * @param values sample.
     * @return none.
     */
    void addSample(T newSample) {
        _data.unshift(newSample);
        _averageValid = _deviationValid = false;
    }

    /**
     * Returns the average of all stored samples.
     * Will calculate all values again if a new sample
     * has been added since last calculation.
     * 
     *
     * @param values none.
     * @return average.
     */
    T getAverage() {
        if (!_averageValid) {
            _updateAverage();
            _averageValid = true;
        }
        return _average;
    }

    /**
     * Returns the deviation of stored samples (accuracy of average).
     * Will calculate all values again if a new sample
     * has been added since last calculation.
     * 
     *
     * @param values none.
     * @return deviation.
     */
    T getDeviation() {
        if (!_deviationValid) {
            _updateDeviation(getAverage());
            _deviationValid = true;
        }
        return _deviation;
    }


    /**
     * Uses the given sample and calulates 
     * deviation of stored samples from
     * this given sample. 
     * Given sample will not be saved.
     * 
     *
     * @param values sample.
     * @return deviation.
     */
    T getDeviation(float sample) {
        _updateDeviation(sample); //Do not set _deviationValid to true, because the new deviation value does not come from stored samples.
        return _deviation;
    }


private:

    bool _updateAverage() {

        if (_data.size() == 0) return false;

        _average = 0;
        for (uint32_t n = 0; n < _data.size(); n++) {
            _average += _data[n];
        }
        _average /= _data.size();

        return true;

    }

    bool _updateDeviation(float average) {

        if (_data.size() <= 1) return false;

        _deviation = 0;
        for (uint32_t n = 0; n < _data.size(); n++) {
            T x = _data[n] - average;
            _deviation += x*x;
        }
        _deviation = sqrt(_deviation/(_data.size() - 1));

        return true;

    }

    CircularBuffer <T, S> _data;

    T _average;
    T _deviation;

    bool _averageValid = false;
    bool _deviationValid = false;


};











#endif