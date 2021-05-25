#ifndef VALUEERRORH
#define VALUEERRORH



template<typename T = float>
class ValueError {
public:

    ValueError() {
        value = 0;
        error = 0;
    }

    /**
     * Give sampleRate to improve performance
     *
     * @param value Best value
     * @param error Error of value
     * @return none.
     */
    ValueError(T value, T error = 0) {
        value = value;
        error = error;
    }

    //Values
    T value;
    T error;

    //Methodes
    /**
     * Calculates the resulting best value and error when combining.
     * 
     * @param secondValue Value to combine with.
     * @returns ValueError with the most likely best value and its error.
     */
    ValueError weightedAverage(const ValueError &valueB) {

        if (error == 0 && valueB.error == 0) {
            return ValueError((value + valueB.value)/2, 0);
        } else if(error == 0) {
            return *this;
        } else if (valueB.error == 0) {
            return valueB;
        }

        T w1 = 1/(error*error);
        T w2 = 1/(valueB.error*valueB.error);

        return ValueError((value*w1 + valueB.value*w2)/(w1 + w2), 1/sqrt(w1 + w2));

    }

    //Operators
    ValueError& operator = (const ValueError &valueB) {
        value = valueB.value;
        error = valueB.error;
        return *this;
    }

    ValueError operator + (const ValueError &valueB) {
        return ValueError(value + valueB.value, error + valueB.error);
    }

    ValueError operator + (const T &valueB) {
        return ValueError(value + valueB, error);
    }

    ValueError operator - (const ValueError &valueB) {
        return ValueError(value - valueB.value, error + valueB.error);
    }

    ValueError operator - (const T &valueB) {
        return ValueError(value - valueB, error);
    }

    ValueError operator * (const ValueError &valueB) {
        T bestValue = value*valueB.value;
        return ValueError(bestValue, bestValue*(error/value + valueB.error/valueB.value));
    }

    ValueError operator * (const T &valueB) {
        return ValueError(value*valueB, error*valueB);
    }

    ValueError operator / (const ValueError &valueB) {
        T bestValue = value/valueB.value;
        return ValueError(bestValue, bestValue*(error/value + valueB.error/valueB.value));
    }

    ValueError operator / (const T &valueB) {
        return ValueError(value/valueB, error/valueB);
    }

};



#endif