#include "starship.h"



void Starship::thread() {

    if (!vehicleInitialized_) init();


}


void Starship::init() {

    //Setup hover controller
    control_->setAngularVelocityPIDFactors(Vector(1,1,0.1), Vector(0), Vector(0), Vector(1000), true);

    //Mark that vehicle has been initialized.
    vehicleInitialized_ = true;

}