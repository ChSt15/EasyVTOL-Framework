#include "output_template.h"



DynamicData* Output::_dynamicSetpoint = &Output::_dynamicSetpointDefault;
DynamicData Output::_dynamicSetpointDefault;

NavigationData* Output::_navigationData = &Output::_navigationDataDefault;
NavigationData Output::_navigationDataDefault;
