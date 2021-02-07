#ifndef KINETIC_DATA_H
#define KINETIC_DATA_H


#include "definitions.h"

#include "vector_math.h"
#include "quaternion_math.h"


/**
 * Kinetic data is usually in world coordinate system!.
 */
struct KineticData {

    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector linearAcceleration;

    Quaternion attitude = Quaternion(1,0,0,0);
    Vector angularRate;
    
};



#endif