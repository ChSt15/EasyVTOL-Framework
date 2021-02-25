#include "vehicle_template.h"



//Contains current forces exerted on the vehicle.
VEHICLE_MODE Vehicle::_vehicleMode;

//Points to the navigation module to use.
Navigation* Vehicle::_navigation = nullptr;

//Points to the guidance module to use.
Guidance* Vehicle::_guidance = nullptr;
