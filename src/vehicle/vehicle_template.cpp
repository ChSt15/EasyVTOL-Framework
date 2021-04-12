#include "vehicle_template.h"



//Points to the navigation module to use.
Navigation* Vehicle::_navigation = nullptr;

//Points to the guidance module to use.
Guidance* Vehicle::_guidance = nullptr;

//Points to the control module to use.
Control* Vehicle::_control = nullptr;

//Points to the dynamics module to use.
Dynamics* Vehicle::_dynamics = nullptr;

//Points to the output module to use.
//Output* Vehicle::_output = nullptr;
