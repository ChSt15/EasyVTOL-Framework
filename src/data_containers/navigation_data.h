#ifndef NAVIGATION_DATA_H
#define NAVIGATION_DATA_H



#include "kinematic_data.h"



/**
 * This enum is for the navigation attitude solution
 */
enum eNavAttitudeMode_t {
    //No attitude solution.
    eNavAttitudeMode_None,
    //Only angular rate can be measured.
    eNavAttitudeMode_AngularRate,
    //Attitude solution but no heading (No yaw reference).
    eNavAttitudeMode_Attitude,
    //Attitude and heading solution.
    eNavAttitudeMode_AHRS
};

/**
 * This enum is for the navigation position solution
 */
enum eNavPositionMode_t {
    //No position solution.
    eNavPositionMode_None,
    //Solution using only barometer data. No position available. Height accuracy is medium.
    eNavPositionMode_BarometerOnly,
    //Solution using only GNSS data. Position and height available. Height and position accuracy is low.
    eNavPositionMode_GNSSOnly,
    //Solution using both GNSS and barometer data. Position and height available. Height accuracy is medium and position accuracy is medium.
    eNavPositionMode_GNSSAndBarometer,
    //Solution using GNSS and IMU data. Position and height available. Height accuracy is medium and position is high.
    eNavPositionMode_GNSSAndIMU,
    //Solution using barometer and IMU data. Only height available. Height accuracy is high.
    eNavPositionMode_BarometerAndIMU,
    //Solution using GNSS, barometer and IMU data. Position and height available. Height and position accuracy is high.
    eNavPositionMode_GNSSAndBarometerAndIMU,
};


/**
 * Inherits from Kinematic but add ability to show what the current state of attitude and position solution.
 */
struct NavigationData: public KinematicData {

    //Current attitude solution state.
    eNavAttitudeMode_t attitudeMode = eNavAttitudeMode_t::eNavAttitudeMode_None;
    //Current position solution state.
    eNavPositionMode_t positionMode = eNavPositionMode_t::eNavPositionMode_None;
    
};



#endif