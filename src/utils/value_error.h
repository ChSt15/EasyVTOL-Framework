#ifndef VALUEERROR<T>H
#define VALUEERROR<T>H



#include "math.h"


template<typename T = float>
class ValueError {
public:

    ValueError<T>() {
        value = 0;
        error = 0;
    }

    /**
     * Error defualts to 0
     * @param value Best value
     * @return none.
     */
    ValueError<T>(T valueIn) {
        value = valueIn;
        error = 0;
    }

    /**
     * @param value Best value
     * @param error Error of value
     * @return none.
     */
    ValueError<T>(T valueIn, T errorIn) {
        value = valueIn;
        error = errorIn;
    }

    //Best value
    T value;
    //Error of value
    T error;

    //Methodes
    /**
     * Calculates the resulting best value and error when combining.
     * 
     * @param secondValue Value to combine with.
     * @returns ValueError<T> with the most likely best value and its error.
     */
    ValueError<T> weightedAverage(const ValueError<T> &valueB) {

        if (error < 0.003 && valueB.error < 0.003) {
            return ValueError<T>((value + valueB.value)/2, 0);
        } else if(error < 0.003) {
            return *this;
        } else if (valueB.error < 0.003) {
            return valueB;
        }

        if (!(value == value) && !(valueB.value == valueB.value)) return ValueError<T>(0,10000);
        else if (!(value == value)) valueB;
        else if (!(valueB.value == valueB.value)) *this;

        T w1 = 1/(error*error);
        T w2 = 1/(valueB.error*valueB.error);

        return ValueError<T>((value*w1 + valueB.value*w2)/(w1 + w2), 1/sqrt(w1 + w2));

    }

    //Operators
    ValueError<T>& operator = (const ValueError<T> &valueB) {
        value = valueB.value;
        error = valueB.error;
        return *this;
    }

    ValueError<T>& operator += (const ValueError<T> &valueB) {
        *this = *this + valueB;
        return *this;
    }

    ValueError<T>& operator -= (const ValueError<T> &valueB) {
        *this = *this - valueB;
        return *this;
    }

    ValueError<T> operator + (const ValueError<T> &valueB) {
        return ValueError<T>(value + valueB.value, error + valueB.error);
    }

    ValueError<T> operator + (const T &valueB) {
        return ValueError<T>(value + valueB, error);
    }

    ValueError<T> operator - (const ValueError<T> &valueB) {
        return ValueError<T>(value - valueB.value, error + valueB.error);
    }

    ValueError<T> operator - (const T &valueB) {
        return ValueError<T>(value - valueB, error);
    }

    ValueError<T> operator * (const ValueError<T> &valueB) {
        T bestValue = value*valueB.value;
        return ValueError<T>(bestValue, bestValue*sqrt(error/value + valueB.error/valueB.value));
    }

    ValueError<T> operator * (const T &valueB) {
        return ValueError<T>(value*valueB, error*valueB);
    }

    ValueError<T> operator / (const ValueError<T> &valueB) {
        T bestValue = value/valueB.value;
        return ValueError<T>(bestValue, bestValue*(error/value + valueB.error/valueB.value));
    }

    ValueError<T> operator / (const T &valueB) {
        return ValueError<T>(value/valueB, error/valueB);
    }

    bool operator < (const ValueError<T> &valueB) {
        return value < valueB.value;
    }

    bool operator > (const ValueError<T> &valueB) {
        return value > valueB.value;
    }

    bool operator <= (const ValueError<T> &valueB) {
        return value <= valueB.value;
    }

    bool operator >= (const ValueError<T> &valueB) {
        return value >= valueB.value;
    }

    bool operator == (const ValueError<T> &valueB) {
        return value == valueB.value;
    }

};


template<typename T = float>
extern ValueError<T> sqrt(const ValueError<T> &value);


#endif