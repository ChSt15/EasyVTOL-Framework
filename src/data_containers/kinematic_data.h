#ifndef KINEMATIC_DATA_H
#define KINEMATIC_DATA_H



#include "lib/Math-Helper/src/3d_math.h"
#include "data_container_base.h"



/**
 * Container for attitude kinematic data.
 */
class AttitudeData: virtual public DataContainerTimestamped_Base {
public:

    Vector angularAcceleration;
    Vector angularRate;
    Quaternion attitude;

};


/**
 * Container for position kinematic data.
 */
class PositionData: virtual public DataContainerTimestamped_Base {
public:

    Vector acceleration;
    //Acceleration with gravity removed
    Vector linearAcceleration;
    Vector velocity;
    Vector position;

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