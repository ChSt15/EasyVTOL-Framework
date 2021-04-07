#ifndef NAVIGATION_DATA_H
#define NAVIGATION_DATA_H



#include "kinematic_data.h"



/**
 * This enum is for the navigation attitude solution
 */
enum NAV_ATTITUDE_MODE {
    //No attitude solution.
    NO_ATTITUDE_SOLUTION,
    //Only angular rate can be measured.
    ANGULAR_RATE,
    //Attitude solution but no heading (No yaw reference).
    ATTITUDE,
    //Attitude and heading solution.
    AHRS
};

/**
 * This enum is for the navigation position solution
 */
enum NAV_POSITION_MODE {
    //No position solution.
    NO_POSITION_SOLUTION,
    //Solution using only barometer data. No position available. Height accuracy is medium.
    BAROMETER,
    //Solution using only GNSS data. Position and height available. Height and position accuracy is low.
    GNSS,
    //Solution using both GNSS and barometer data. Position and height available. Height accuracy is medium and position accuracy is medium.
    GNSS_BAROMETER,
    //Solution using GNSS and IMU data. Position and height available. Height accuracy is medium and position is high.
    GNSS_IMU,
    //Solution using barometer and IMU data. Only height available. Height accuracy is high.
    BAROMETER_IMU,
    //Solution using GNSS, barometer and IMU data. Position and height available. Height and position accuracy is high.
    GNSS_BAROMETER_IMU,
};


/**
 * Inherits from Kinematic but add ability to show what the current state of attitude and position solution.
 */
struct NavigationData: public KinematicData {

    //Current attitude solution state.
    NAV_ATTITUDE_MODE attitudeMode = NAV_ATTITUDE_MODE::NO_ATTITUDE_SOLUTION;
    //Current position solution state.
    NAV_POSITION_MODE positionMode = NAV_POSITION_MODE::NO_POSITION_SOLUTION;
    
};



#endif