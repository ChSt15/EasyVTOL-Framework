#include "quaternion_math.h"


template <typename T = float>
Quaternion<T> sqrt(const Quaternion<T> &a) {
    return Quaternion<T>(sqrtf(a.w), sqrtf(a.x), sqrtf(a.y), sqrtf(a.z));
}