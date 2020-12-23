#include "imu_template.h"


namespace IMUTemplate {

    namespace {

        IntervalControl imuInterval; //This is private

        /*
         * Here are where all variables must be stored if they are not to be seen from outside of 
         * the imu.h/cpp files (private)
         * If they are to be seen from outside (public) then declare them below outside of the unnamed namespace
        */

    }


    DeviceStatus imuStatus = DeviceStatus::DEVICE_NOT_STARTED; // This is public

}


void IMUTemplate::imuThread() { //This is public because it was declare in the imu.h file

    if (!imuInterval.isTimeToRun()) return; //This limits the rate the thread will run at to save time. By default the rate is infinite until the code below resticts it (pls do this).


    if (imuStatus == DeviceStatus::DEVICE_RUNNNIG) {

        /*
         * Here is where all the code goes for running the device, checking for error etc.
         * If an error occures then go into failure mode.
        */

    } else if (imuStatus == DEVICE_NOT_STARTED) {

        imuInterval.setRate(1000); // Set the maximum rate the device loop should be run at. Here at 1000Hz

        /*
         * Start the device here and check if it was successfull. If so then continue by going into calibration if
         * needed or go directly into running. If it fails then go into failure mode.
        */

    } else if (imuStatus == DeviceStatus::DEVICE_FAILURE) {

        /*
         * This is where all the code goes if an error occures.
         * Do what ever is need if device failes. e.g. if the imu failes then the 
         * ENTIRE system must shut down as this is a dangerous situation
        */

    }


    /*
     * Further if else statements can be added but keep in mind the way everything
     * is ordered as above. This why the proccessor only has to make one comparison
     * to enter the code not multiple. This improves perfomance.
    */


}


