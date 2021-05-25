#ifndef NAVIGATION_DATA_H
#define NAVIGATION_DATA_H



#include "kinematic_data.h"



namespace {

    const double c_earthRadius = 6371.0e3;
    const double c_earthMass = 5.9722e24;
    
}



/**
 * This enum is for the navigation attitude solution
 */
enum eNavAttitudeMode_t: uint8_t {
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
enum eNavPositionMode_t: uint8_t {
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
 * Contains position data used for world coordinates.
 * Usefull for setting home points.
 */
struct WorldPosition {
    //In radians
    double latitude;
    //In radians
    double longitude;
    //In meters above MSL
    float height;

    /**
     * This is fairly computational due to usage of double values.
     * 
     * This calculates the position in reference to the given reference point. 
     * X is distance towards north from reference. 
     * Y is distance towards West from reference. 
     * Z is height above reference. 
     * 
     * @param WorldPosition Reference point for position
     * @returns position vector
     */
    Vector<> getPositionVectorFrom(const WorldPosition &referencePoint) {

        float heightRef = height - referencePoint.height;

        double radius = c_earthRadius + height;
        float cosLat = cosf(referencePoint.latitude); 

        float north = (latitude - referencePoint.latitude)*radius;
        float west = (longitude - referencePoint.longitude)*radius*cosLat;

        return Vector<>(north, west, heightRef);

    }

};


/**
 * Inherits from Kinematic but add ability to show what the current state of attitude and position solution.
 */
struct NavigationData: public KinematicData {

    //Current home position. Output is in reference to this position
    WorldPosition homePosition;

    //Current absolute position
    WorldPosition absolutePosition;

    //Current attitude solution state.
    eNavAttitudeMode_t attitudeMode = eNavAttitudeMode_t::eNavAttitudeMode_None;
    //Current position solution state.
    eNavPositionMode_t positionMode = eNavPositionMode_t::eNavPositionMode_None;
    
};



#endif