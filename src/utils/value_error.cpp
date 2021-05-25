#include "value_error.h"



template<typename T = float>
ValueError<T> sqrt(const ValueError<T> &value) {
    return ValueError<T>(sqrt(value.value), 1/(2*sqrt(value.value)*value.error));
}