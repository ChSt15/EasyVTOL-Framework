#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "Arduino.h"

#include "templates/module_template.h"

#include "data_containers/kinematic_data.h"

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



class Navigation: public Module {
public:

    /**
     * Returns a struct containing all the vehicles
     * current kinematic parameters.
     *
     * @param values none.
     * @return Kinetic paramenters.
     */
    virtual KinematicData getKinematicData() {return _vehicleKinematics;};

    /**
     * Returns the current position accuracy.
     * Returns -1.0 if unsupported.
     * 
     * @param values none.
     * @return float.
     */
    virtual float getPositionAccuracy() {return -1.0f;}

    /**
     * Inits the nav module and give it the pointer
     * it updates with the new kinematic data.
     * 
     * 
     * @param values dataPointer.
     * @return none.
     */
    //void init(KinematicData* dataPointer) {_vehicleKinematics = dataPointer; init();}


protected:

    /**
     * This pointer points to the global vehicle data kinematic data container
     */
    static KinematicData _vehicleKinematics;

    /**
     * Current attitude solution state. 
     */
    NAV_ATTITUDE_MODE _attitudeMode;

    /**
     * Current position solution state. 
     */
    NAV_POSITION_MODE _positionMode;


};




#endif