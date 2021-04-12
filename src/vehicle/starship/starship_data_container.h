#ifndef STARSHIP_DATA_H
#define STARSHIP_DATA_H



#include "data_containers/vehicle_data.h"



/**
 * Starship specific data container.
 */
struct StarshipData: public VehicleData {

    //If true then flip to belly flop position and mainly use flaps for control.
    bool bellyFlop = false;

};



#endif