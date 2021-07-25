#include "KraftKontrol/modules/module_abstract.h"



Topic<KraftMessageContainer> Module_Abstract::globalMessages_;
Topic<KraftMessageContainer> Module_Abstract::telemetryMessages_;

List<Module_Abstract*> Module_Abstract::existingModules_;