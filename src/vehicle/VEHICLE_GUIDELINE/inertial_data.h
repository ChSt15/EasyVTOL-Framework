#ifndef VEHICLE_H
#define VEHICLE_H


#include "definitions.h"

#include "vector_math.h"
#include "quaternion_math.h"


struct InertialData {

    Vector _position;
    Vector _velocity;
    Vector _acceleration;
    Vector _linearAcceleration;

    Quaternion _attitude = Quaternion(1,0,0,0);
    Vector _angularRate;
    
};



#endif