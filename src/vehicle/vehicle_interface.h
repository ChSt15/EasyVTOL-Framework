#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "data_containers/kinematic_data.h"
#include "data_containers/vehicle_data.h"

#include "modules/guidance_modules/guidance_interface.h"
#include "modules/navigation_modules/navigation_interface.h"
#include "modules/control_modules/control_interface.h"
#include "modules/dynamics_modules/dynamics_interface.h"



class Vehicle_Interface {
public:

    /**
     * Returns pointer to the navigation module the vehicle uses. 
     * @returns Navigation_Interface
     */
    virtual Navigation_Interface* getNavigationModulePointer() = 0;

    /**
     * Returns pointer to the guidance module the vehicle uses. 
     * @returns Guidance_Interface
     */
    virtual Guidance_Interface* getGuidanceModulePointer() = 0;

    /**
     * Returns pointer to the control module the vehicle uses. 
     * @returns Control_Interface
     */
    virtual Control_Interface* getControlModulePointer() = 0;

    /**
     * Returns pointer to the dynamics module the vehicle uses. 
     * @returns Dynamics_Interface
     */
    virtual Dynamics_Interface* getDynamicsModulePointer() = 0;

    /**
     * Arms vehicle. Vehicle will ready itsself and then go into flight mode.
     */
    virtual void armVehicle() = 0;

    /**
     * Disarms vehicle. Vehicle should shutdown immediately
     */
    virtual void disarmVehicle() = 0;

    /**
     * Gets the vehicle data.
     * 
     * @returns VehicleData of vehicle
     */
    virtual VehicleData getVehicleData() = 0;


};



#endif