#include "vehicle_template.h"



//Contains current vehicle movement.
KinematicData Vehicle::_vehicleKinematics;

//Contains current forces exerted on the vehicle.
VEHICLE_MODE Vehicle::_vehicleMode;

//Points to the navigation module to use.
Navigation* Vehicle::_navigation;

//Points to the guidance module to use.
Guidance* Vehicle::_guidance;
