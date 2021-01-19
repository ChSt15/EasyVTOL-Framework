#include "servo_ppm.h"



void PPMChannel::deviceThread() {

    if (!servoInterval.isTimeToRun()) return; 

    

}



DeviceStatus PPMChannel::getDeviceStatus() {return servoStatus;}


