#include "dynamics_template.h"



KinematicData* Dynamics::_kinematicSetpoint = &Dynamics::_kinematicSetpointDefault;
KinematicData Dynamics::_kinematicSetpointDefault;
DynamicData Dynamics::_dynamicOutput;

NavigationData* Dynamics::_navigationData = &Dynamics::_navigationDataDefault;
NavigationData Dynamics::_navigationDataDefault;
