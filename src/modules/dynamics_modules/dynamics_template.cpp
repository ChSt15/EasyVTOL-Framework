#include "dynamics_template.h"



DynamicData Dynamics::_dynamicsSetpointDefault;
DynamicData* Dynamics::_dynamicSetpoint = &_dynamicsSetpointDefault;

NavigationData Dynamics::_navigationDataDefault;
NavigationData* Dynamics::_navigationData = &Dynamics::_navigationDataDefault;

