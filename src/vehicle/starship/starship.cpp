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
    _navigation->init(); //init the navigation module and pass pointer to vehicle data.
    _guidance->init();
}