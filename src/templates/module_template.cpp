#include "module_template.h"



uint16_t Module::_threadCount = 0;
Module* Module::_nextModuleToRun = 0;
Module* Module::_firstModule = nullptr;