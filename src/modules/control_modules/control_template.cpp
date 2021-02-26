#include "control_template.h"



ControlData* Control::_controlSetpoint = &Control::_controlSetpointDefault;
NavigationData* Control::_navigationData = &Control::_navigationDataDefault;

ControlData Control::_controlSetpointDefault;
NavigationData Control::_navigationDataDefault;

KinematicData Control::_controlOutput;
