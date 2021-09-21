#include "KraftKontrol/modules/module_abstract.h"



Topic<KraftMessage_Interface>& Module_Abstract::globalMessages() {

    static Topic<KraftMessage_Interface> g_globalMessages;

    return g_globalMessages;

}


/*Topic<KraftMessageContainer>& Module_Abstract::telemetryMessages() {

    static Topic<KraftMessageContainer> g_telemetryMessages;

    return g_telemetryMessages;

}*/


List<Module_Abstract*>& Module_Abstract::existingModules() {

    static List<Module_Abstract*> g_existingModules;

    return g_existingModules;

}

//bool Module_Abstract::isSimulation_ = false;