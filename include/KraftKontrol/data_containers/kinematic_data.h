#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H



#include "lib/Math-Helper/src/3d_math.h"
#include "data_container_base.h"



/**
 * Container for attitude kinematic data.
 */
class AttitudeData: virtual public DataContainerTimestamped_Base {
public:

    Vector<> angularAcceleration;
    Vector<> angularAccelerationError = 0;

    Vector<> angularRate;
    Vector<> angularRateError = 0;

    Quaternion<> attitude = Quaternion<>(1,0,0,0);
    Quaternion<> attitudeError = 0;

};



/**
 * Container for position kinematic data.
 */
class PositionData: virtual public DataContainerTimestamped_Base {
public:

    //Total acceleration
    Vector<> acceleration;
    Vector<> accelerationError = 0;
    //Acceleration with gravity removed. Error is the same as accelerationError
    Vector<> linearAcceleration;

    Vector<> velocity;
    Vector<> velocityError = 0;

    Vector<> position;
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
    KinematicData getStatePrediction(const uint32_t &dTime) {
        return *this;
    }
    
};



#endif