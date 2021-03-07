#ifndef KRAFT_KONTROL_H
#define KRAFT_KONTROL_H



#include "definitions.h"

#include "sensors/imu.h"
#include "sensors/air_data.h"
#include "sensors/gps.h"

#include "comms/lora_2_4.h"

#include "outputs/rgb_led.h"

#include "vehicle/vehicle_template.h"
#include "vehicle_control/control_profile_template.h"

#include "utils/interval_control.h"

#include "scheduler.h"



#define WAIT_FOR_SERIAL //Tells system to wait for a signal from the usb serial port before continueing



namespace KraftKontrol {

    //This is the vehicle that will be use by KraftKontrol. Set it to a reference of the wanted vehicle.
    extern Vehicle* kraft;

    //This is the vehicle that will be use by KraftKontrol. Set it to a reference of the wanted vehicle.
    extern ControlProfile* controlProfile;

    void initialise();

    void loop();

}



#endif