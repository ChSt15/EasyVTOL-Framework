#ifndef KRAFT_KONTROL_H
#define KRAFT_KONTROL_H



#include "vehicle/vehicle_interface.h"



//#define WAIT_FOR_SERIAL //Tells system to wait for a signal from the usb serial port before continueing



class KraftKontrolRunner {
public:

    KraftKontrolRunner(void (*vehicleProgram)(uint32_t* runCounter));


};



#endif