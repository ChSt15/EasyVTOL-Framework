#include "manual_profile.h"



void ManualControlProfile::thread() {

    if (_vehicle == nullptr) return; //Leave if no vehicle given.

    _vehicle->setGuidancePointer(&_guidanceFBW); //Set the vehicles guidance to manual one.

    if (!IBUSReceiver::channelDataFifo.isEmpty()) {

        RCChannelData _channelData = IBUSReceiver::channelDataFifo.pop();

        if (_channelData.failsafe) {
            *_vehicleMode = VEHICLE_MODE::MODE_FAILSAFE;
        } else {

            if (*_vehicleMode == VEHICLE_MODE::MODE_FAILSAFE) {
                if (_channelData.channelData[IBUS_CHANNEL_SWD] > 0.95f) {
                    *_vehicleMode == VEHICLE_MODE::MODE_DISARM; 
                }
            } else if (*_vehicleMode == VEHICLE_MODE::MODE_DISARM) {
                if (_channelData.channelData[IBUS_CHANNEL_THROTTLE] < -0.95f && _channelData.channelData[IBUS_CHANNEL_SWD] < -0.95f && _vehicle->vehicleReady()) {
                    *_vehicleMode == VEHICLE_MODE::MODE_ARM; 
                }
            } else if (*_vehicleMode == VEHICLE_MODE::MODE_ARM) {

                if (_channelData.channelData[IBUS_CHANNEL_SWD] > 0.95f) {
                    *_vehicleMode == VEHICLE_MODE::MODE_DISARM; 
                }
                
                Vector angularRate;
                Vector velocity(0);

                const float maxAngX = 600*DEGREES; 
                const float maxAngY = 600*DEGREES; 
                const float maxAngZ = 200*DEGREES; 

                const float maxVelZ = 3; 

                angularRate.x = _channelData.channelData[IBUS_CHANNEL_ROLL]*maxAngX;
                angularRate.y = _channelData.channelData[IBUS_CHANNEL_PITCH]*maxAngY;
                angularRate.z = _channelData.channelData[IBUS_CHANNEL_YAW]*maxAngZ;

                velocity.z = _channelData.channelData[IBUS_CHANNEL_THROTTLE]*maxVelZ;

                velocity = _vehicle->getNavigationData().attitude.rotateVector(velocity);

                _guidanceFBW.setAngularRate(angularRate);
                _guidanceFBW.setVelocity(velocity);

                _guidanceFBW.setAttitudeControlMode(CONTROL_MODE::CONTROL_VELOCITY);
                _guidanceFBW.setPositionControlMode(CONTROL_MODE::CONTROL_DISABLED);

            }

        }

    }



}


void ManualControlProfile::init() {

    

}