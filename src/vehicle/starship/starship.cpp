#include "starship.h"



void Starship::thread() {

    if (!_interval.isTimeToRun()) return; //Leave if its not time to run yet

    if (!_vehicleInitialized) init();

    _navigation->thread(); //Run navigation thread

    _guidance->thread(); //Run guidance thread

}


void Starship::init() {
    _navigation->init(); //init the navigation module and pass pointer to vehicle data.
    _guidance->init();
}