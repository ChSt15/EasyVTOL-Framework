#include "vector_math.h"



template <typename T = float>
Vector<T> sqrt(const Vector<T> &a) {
    return Vector<T>(sqrt(a.x), sqrt(a.y), sqrt(a.z));
}
