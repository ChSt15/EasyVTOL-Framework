#include "kwadcopter.h"



void Kwadcopter::thread() {

    if (!_vehicleInitialized) init(); //Initialise vehiclein not yet done.

    _navigation->thread(); //Run navigation thread

    if (_guidance == &_guidanceFBW) { //Check if we are using FBW guidance module. If so then do stuff
        //_guidanceFBW.setAngularRate(Vector(cos((float)millis()/1000.0f),0,0));
    }

    _guidance->thread(); //Run guidance thread

    _control->thread(); //Run control thread

    _dynamics->thread(); //Run dynamics thread

}


void Kwadcopter::init() {

    //link module data together
    _control->linkControlSetpointPointer(_guidance->getControlSetpointPointer()); //Guidance -> Control
    _control->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Navigation -> Control
    _dynamics->linkKinematicSetpointPointer(_control->getKinematicOutputPointer()); //Navigation -> Dynamics
    _dynamics->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Control -> Dynamics
    //_output->linkNavigationDataPointer(_navigation->getNavigationDataPointer());
    //_output->linkDynamicSetpointPointer(_dynamics->getDynamicSetpointPointer());

    //Initialise all modules
    _navigation->init(&_vehicleMode);
    _guidance->init(&_vehicleMode);
    _control->init(&_vehicleMode);
    _dynamics->init(&_vehicleMode);
    //_output->init();

    _vehicleInitialized = true;

}