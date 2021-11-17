#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H



#include "lib/Math-Helper/src/3d_math.h"



/**
 * Container for attitude kinematic data.
 */
class AttitudeData {
public:

    Vector<> angularAcceleration = 0;
    Vector<> angularAccelerationError = 0;

    Vector<> angularRate = 0;
    Vector<> angularRateError = 0;

    Quaternion<> attitude = Quaternion<>(1,0,0,0);
    Quaternion<> attitudeError = 0;

};



/**
 * Container for position kinematic data.
 */
class PositionData {
public:

    //Total acceleration
    Vector<> acceleration = 0;
    Vector<> accelerationError = 0;
    //Acceleration with gravity removed. Error is the same as accelerationError
    Vector<> linearAcceleration;

    Vector<> velocity = 0;
    Vector<> velocityError = 0;

    Vector<> position = 0;
    Vector<> positionError = 0;

};



/**
 * Container for attitude and position kinematic data.
 */
class KinematicData: public PositionData, public AttitudeData {
public:

    /**
     * Uses data to predict next state after dTime
     * @param dTime Amount of time to predict in micros
     */
    /*KinematicData getStatePrediction(int64_t dTime) {
        return *this;
    }*/
    
};



#endif