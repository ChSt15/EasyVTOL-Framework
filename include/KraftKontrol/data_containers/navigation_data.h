#ifndef NAVIGATION_DATA_H
#define NAVIGATION_DATA_H



#include "kinematic_data.h"



namespace {

    const double c_earthRadius = 6371.0*1000;
    const double c_earthMass = 5.9722e24;
    
}



/**
 * This enum is for the navigation attitude solution
 */
class NavAttitudeMode {
public:

    NavAttitudeMode() {
        attitudeValid = false;
        headingCorrect = false;
    }

    //True if attitude is valid. Roll/Pitch reference to Gravity.
    bool attitudeValid = false;

    //True if heading is Valid. Yaw in reference to North.
    bool headingCorrect = false;

};


/**
 * This enum is for the navigation position solution
 */
class NavPositionMode {
public:

    NavPositionMode() {
        heightValid = false;
        positionValid = false;
    }
    
    //True if height is valid and can be used.
    bool heightValid = false;

    //True if horizontal position is valid and can be used.
    bool positionValid = false;

};


/**
 * Contains position data used for world coordinates.
 * Usefull for setting home points.
 */
struct WorldPosition {
    //In radians
    double latitude = 0;
    //In radians
    double longitude = 0;
    //In meters above MSL
    float height = 0;

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
    Vector<> getPositionVectorFrom(const WorldPosition &referencePoint) const {

        double heightRef = height - referencePoint.height;

        double radius = c_earthRadius + height;
        double cosLat = cos(referencePoint.latitude); 

        double north = (latitude - referencePoint.latitude)*radius;
        double east = (longitude - referencePoint.longitude)*radius*cosLat;

        return Vector<>(north, -east, heightRef);

        /*
        //Calculate helper variables
        double radiusA = c_earthRadius + height;
        double radiusB = c_earthRadius + referencePoint.height;

        //Calculate ECEF xyz coodinates
        float aX = radiusA*cos(latitude)*cos(longitude);
        float aY = radiusA*cos(latitude)*sin(longitude);
        float aZ = radiusA*sin(latitude);

        float bX = radiusB*cos(referencePoint.latitude)*cos(referencePoint.longitude);
        float bY = radiusB*cos(referencePoint.latitude)*sin(referencePoint.longitude);
        float bZ = radiusB*sin(referencePoint.latitude);

        Vector<> bToA = Vector<>(float(aX-bX), float(aY-bY), float(aZ-bZ));

        //Rotate vector onto tangential plane on references coordinates
        Quaternion<> rotation = Quaternion<>(Vector<>(0,0,1), referencePoint.longitude)*Quaternion<>(Vector<>(0,1,0), referencePoint.latitude);
        bToA = rotation.rotateVector(bToA);

        //Change to system coordinate system
        Vector<> buf = bToA;

        bToA.x = buf.z;
        bToA.y = -buf.y;
        bToA.z = buf.x;

        return bToA;*/

    }

};



/**
 * Inherits from NavPositionData and NavAttitudeData to combine them and add other important information.
 */
class NavigationData: public KinematicData {
public:

    NavigationData() {}

    NavigationData(const KinematicData &kinematicData) {

        angularAcceleration = kinematicData.angularAcceleration;
        angularAccelerationError = kinematicData.angularAccelerationError;
        angularRate = kinematicData.angularRate;
        angularRateError = kinematicData.angularRateError;
        attitude = kinematicData.attitude;
        attitudeError = kinematicData.attitudeError;

        acceleration = kinematicData.acceleration;
        accelerationError = kinematicData.accelerationError;
        linearAcceleration = kinematicData.linearAcceleration;
        velocity = kinematicData.velocity;
        velocityError = kinematicData.velocityError;
        position = kinematicData.position;
        positionError = kinematicData.positionError;

    }


    //Current attitude solution state.
    NavAttitudeMode attitudeMode;

    //Current position solution state.
    NavPositionMode positionMode;

    //Current home position. Output is in reference to this position
    WorldPosition homePosition;

    //Current absolute position
    WorldPosition absolutePosition;

    
};



#endif