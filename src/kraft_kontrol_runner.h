#ifndef KRAFT_KONTROL_H
#define KRAFT_KONTROL_H



#include "definitions.h"

#include "modules/sensor_modules/mpu9250_driver.h"
#include "modules/sensor_modules/bme280_driver.h"
#include "sensors/gps.h"

#include "outputs/rgb_led.h"

#include "vehicle/vehicle_template.h"
#include "flight_profiles/control_profile_template.h"

#include "utils/interval_control.h"

#include "scheduler.h"



//#define WAIT_FOR_SERIAL //Tells system to wait for a signal from the usb serial port before continueing



namespace KraftKontrolRunner {

    //This is the vehicle that will be use by KraftKontrol. Set it to the reference of the to be controlled vehicle.
    extern Vehicle* kraft;

    //This is the control profile that will be use by KraftKontrol. Set it to the reference of the to be used control profile.
    extern ControlProfile* controlProfile;

    //This is the scheduler used by kraftkontrol. Only make changes if you really know what you're doing
    extern Scheduler systemScheduler;

    void initialise();

    void loop();

}



#endif