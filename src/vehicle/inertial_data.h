#ifndef INERTIAL_DATA_H
#define INERTIAL_DATA_H


#include "definitions.h"

#include "vector_math.h"
#include "quaternion_math.h"


struct InertialData {

    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector linearAcceleration;

    Quaternion attitude = Quaternion(1,0,0,0);
    Vector angularRate;
    
};



#endif