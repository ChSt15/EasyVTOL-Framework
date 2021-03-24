#include "kwadcopter.h"



void Kwadcopter::thread() {

    if (!_vehicleInitialized) init(); //Initialise vehiclein not yet done.

    if (_navigation != nullptr) _navigation->thread(); //Run navigation thread. Check to make sure it isnt invalid

    if (_guidance != nullptr) _guidance->thread(); //Run guidance thread. Check to make sure it isnt invalid

    if (_control != nullptr) _control->thread(); //Run control thread. Check to make sure it isnt invalid

    if (_dynamics != nullptr) _dynamics->thread(); //Run dynamics thread. Check to make sure it isnt invalid

}


void Kwadcopter::init() {

    //link module data together
    _control->linkControlSetpointPointer(_guidance->getControlSetpointPointer()); //Guidance -> Control
    _control->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Navigation -> Control
    //_dynamics->linkKinematicSetpointPointer(_control->getKinematicOutputPointer()); //Navigation -> Dynamics
    _dynamics->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Control -> Dynamics

    //Initialise all modules
    _navigation->init(&_vehicleMode);
    _guidance->init(&_vehicleMode);
    _control->init(&_vehicleMode);
    _dynamics->init(&_vehicleMode);
    //_output->init();

    _vehicleInitialized = true;

}