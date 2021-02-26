#include "starship.h"



void Starship::thread() {

    if (!_interval.isTimeToRun()) return; //Leave if its not time to run yet

    if (!_vehicleInitialized) init();

    _navigation->thread(); //Run navigation thread

    if (_guidance == &_guidanceFBW) { //Check if we are using FBW guidance module
        //_guidanceFBW.setAngularRate(Vector(cos((float)millis()/1000.0f),0,0));
    }

    _guidance->thread(); //Run guidance thread

}


void Starship::init() {

    //link module data together
    _control->linkControlSetpointPointer(_guidance->getControlSetpointPointer()); //Links the data together
    _control->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Links the data together
    _dynamics->linkKinematicSetpointPointer(_control->getKinematicOutputPointer()); //Links the data together
    _output->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Links the data together
    _output->linkDynamicSetpointPointer(_dynamics->getDynamicSetpointPointer()); //Links the data together

    //Initialise all modules
    _navigation->init(); //init the navigation module and pass pointer to vehicle data.
    _guidance->init();
    _control->init();
    _dynamics->init();
    _output->init();

}