#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H


#include "definitions.h"

#include "vector_math.h"
#include "quaternion_math.h"


/**
 * Kinetic data is usually in world coordinate system!.
 */
struct KinematicData {

    //Position in meters.
    Vector position;
    //Velocity in m/s.
    Vector velocity;
    //Acceleration in m/s/s.
    Vector acceleration;
    //Acceleration without gravity.
    Vector linearAcceleration;

    //Attitude is without units as it is a quaternion. No idea what unit it should have ¯\_(ツ)_/¯
    Quaternion attitude = Quaternion(1,0,0,0);
    //Angular rate in radians/s.
    Vector angularRate;
    
};



#endif