#include "quaternion_math.h"


Quaternion sqrt(const Quaternion &a) {
    return Quaternion(sqrtf(a.w), sqrtf(a.x), sqrtf(a.y), sqrtf(a.z));
}