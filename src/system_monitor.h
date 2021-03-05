#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H



#include "definitions.h"

#include "sensors/imu.h"
#include "sensors/air_data.h"
#include "sensors/gps.h"

#include "comms/lora_2_4.h"

#include "outputs/rgb_led.h"

#include "kraft_kontrol.h"





void threadSystemMonitor() {

    Vehicle* vehicle = KraftKontrol::kraft;

    KinematicData vehicleKinetics = vehicle->getNavigationData();
    KinematicData vehicleSetpoints = vehicle->getGuidanceData();

    //Serial.println();
    Serial.println("LoopRate: " + String(IMU::getLoopRate()) + ", GyroRate: " + String(IMU::getGyroRate()) + ", AccelRate: " + String(IMU::getAccelRate()) + ", MagRate: " + String(IMU::getMagRate()));
    Serial.println("vehicle attitude: w: " + String(vehicleKinetics.attitude.w) + ", x: " + String(vehicleKinetics.attitude.x) + ", y: " + String(vehicleKinetics.attitude.y) + ", z: " + String(vehicleKinetics.attitude.z));
    //Serial.println("vehicle angularRate: x: " + String(vehicleKinetics.angularRate.x) + ", y: " + String(vehicleKinetics.angularRate.y) + ", z: " + String(vehicleKinetics.angularRate.z));
    //Serial.println("vehicle angularRate setpoint: x: " + String(vehicleSetpoints.angularRate.x) + ", y: " + String(vehicleSetpoints.angularRate.y) + ", z: " + String(vehicleSetpoints.angularRate.z));
    //Serial.println("vehicle attitude setpoint: w: " + String(vehicleSetpoints.attitude.w) + ", x: " + String(vehicleSetpoints.attitude.x) + ", y: " + String(vehicleSetpoints.attitude.y) + ", z: " + String(vehicleSetpoints.attitude.z));
    //Serial.println("IMU Rate: " + String(IMU::getGyroRate()));
    //Serial.println("vehicle accel: x: " + String(vehicle.getAcceleration().x) + ", y: " + String(vehicle.getAcceleration().y) + ", z: " + String(vehicle.getAcceleration().z));
    //Serial.println("vehicle speed: x: " + String(vehicle.getVelocity().x) + ", y: " + String(vehicle.getVelocity().y) + ", z: " + String(vehicle.getVelocity().z));
    //Serial.println("vehicle pos: x: " + String(vehicle.getPosition().x) + ", y: " + String(vehicle.getPosition().y) + ", z: " + String(vehicle.getPosition().z));

    if (Serial.available()) {
        //vehicle.resetInertial();
        while(Serial.available()) Serial.read();
    }

}



#endif