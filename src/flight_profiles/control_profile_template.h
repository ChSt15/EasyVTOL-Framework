#ifndef CONTROL_PROFILE_TEMPLATE_H
#define CONTROL_PROFILE_TEMPLATE_H



#include "vehicle/vehicle_template.h"



class ControlProfile {
public:

    ControlProfile(Vehicle* vehiclePointer) {
        _vehiclePointer = vehiclePointer;
    }


    void gotoPointRelative(Vector position);


    void gotoPointAbsolute(Vector position);


    void waitForTimeMicroseconds(uint32_t delay_microseconds);


    void circlePointForTime(Vector circle_middle, float radius, uint32_t time_microseconds);


    void circlePointForRadiansFromPoint(Vector circle_middle, Vector circle_start, float radians);


    void circle3Point(Vector circle_point1, Vector circle_point2, Vector circle_point3, float radians = 2*PI);



protected:

    Vehicle* _vehiclePointer = nullptr;


};





#endif