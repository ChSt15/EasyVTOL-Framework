#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H



#include "lib/MathHelperLibrary/vector_math.h"
#include "lib/MathHelperLibrary/FML.h"



/**
 * Container for attitude kinematic data.
 */
class AttitudeData {
public:

    VectorOLD<> angularAcceleration = 0;
    VectorOLD<> angularAccelerationError = 0;

    VectorOLD<> angularRate = 0;
    VectorOLD<> angularRateError = 0;

    FML::Quaternion<> attitude = FML::Quaternion<>(1,0,0,0);
    FML::Quaternion<> attitudeError = 0;

};



/**
 * Container for position kinematic data.
 */
class PositionData {
public:

    //Total acceleration
    VectorOLD<> acceleration = 0;
    VectorOLD<> accelerationError = 0;
    //Acceleration with gravity removed. Error is the same as accelerationError
    VectorOLD<> linearAcceleration;

    VectorOLD<> velocity = 0;
    VectorOLD<> velocityError = 0;

    VectorOLD<> position = 0;
    VectorOLD<> positionError = 0;

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