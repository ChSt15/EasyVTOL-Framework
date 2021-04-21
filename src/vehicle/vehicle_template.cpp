#include "vehicle_template.h"



void Vehicle::thread() {

    if (!_vehicleInitialized) init(); //Initialise vehicle if not yet done.

    _scheduler.tick();

}


void Vehicle::init() {

    //link module data together
    _control->linkControlSetpointPointer(_guidance->getControlSetpointPointer()); //Guidance -> Control
    _control->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Navigation -> Control
    _dynamics->linkDynamicSetpointPointer(_control->getDynamicsOutputPointer()); //Navigation -> Dynamics
    _dynamics->linkNavigationDataPointer(_navigation->getNavigationDataPointer()); //Control -> Dynamics

    //Initialise all modules
    _navigation->init();
    _guidance->init();
    _control->init();
    _dynamics->init();

    

    //Mark that vehicle has been initialized.
    _vehicleInitialized = true;

}



void Vehicle::initScheduler(Guidance* guidancePointer, Navigation* navigationPointer, Control* controlPointer, Dynamics* dynamicsPointer) {

    //Add core sensor and comms functions to scheduler 
    _scheduler.attachFunction([] {IMU.thread();}, IMU.get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_REALTIME);
    _scheduler.attachFunction([] {Baro.thread();}, Baro.get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_HIGH);
    //_scheduler.attachFunction(LORA_2_4::deviceThread, 500, TASK_PRIORITY::PRIORITY_MIDDLE);
    //_scheduler.attachFunction(GPS::deviceThread, 100, TASK_PRIORITY::PRIORITY_HIGH);
    //_scheduler.attachFunction(RGBLED::deviceThread, 100, TASK_PRIORITY::PRIORITY_NONE);

    //Add vehicle modules to scheduler
    _scheduler.attachFunction([] (void* navigationPointer) {if (navigationPointer != nullptr) static_cast<Navigation*>(navigationPointer)->thread();}, navigationPointer->get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_VERYHIGH);
    _scheduler.attachFunction([] {if (guidancePointer != nullptr) guidancePointer->thread();}, guidancePointer->get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_VERYHIGH);
    _scheduler.attachFunction([] {if (controlPointer != nullptr) controlPointer->thread();}, controlPointer->get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_VERYHIGH);
    _scheduler.attachFunction([] {if (dynamicsPointer != nullptr) dynamicsPointer->thread();}, dynamicsPointer->get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_VERYHIGH);

    //Initialise scheduler
    _scheduler.initialise();

}
