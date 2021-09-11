#include "KraftKontrol/modules/actuator_modules/actuator_abstract.h"




List<Actuator_Abstract*>& Actuator_Abstract::actuatorList() {

    static List<Actuator_Abstract*> g_actuatorList;

    return g_actuatorList;

}