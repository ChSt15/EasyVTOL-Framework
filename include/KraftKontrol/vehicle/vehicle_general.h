#ifndef VEHICLE_GENERAL_H
#define VEHICLE_GENERAL_H



#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/data_containers/kinematic_data.h"
#include "KraftKontrol/data_containers/vehicle_data.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "KraftKontrol/modules/guidance_modules/guidance_interface.h"
#include "KraftKontrol/modules/navigation_modules/navigation_interface.h"
#include "KraftKontrol/modules/control_modules/control_interface.h"
#include "KraftKontrol/modules/dynamics_modules/dynamics_interface.h"

#include "KraftKontrol/vehicle/vehicle_interface.h"



class VehicleGeneral: public Vehicle_Interface, public Module_Abstract, public Task_Abstract {
public:

    VehicleGeneral(Guidance_Interface* guidancePointer, Navigation_Interface* navigationPointer, Control_Interface* controlPointer, Dynamics_Interface* dynamicsPointer) : Task_Abstract("Vehicle General", 8000, eTaskPriority_t::eTaskPriority_High) {
        guidance_ = guidancePointer;
        navigation_ = navigationPointer;
        control_ = controlPointer;
        dynamics_ = dynamicsPointer;
    }


    /**
     * Thread function of the vehicle. 
     * All calculations the vehicle ever has to do for its 
     * control will be done here.
     *
     * @param values none.
     * @return none.
     */
    virtual void thread() {}

    /**
     * Not really needed for modules like these, but still needs to be defined.
     */
    void removal() {}

    /**
     * Returns pointer to the navigation module the vehicle uses. 
     * @returns Navigation_Interface
     */
    Navigation_Interface* getNavigationModulePointer() {return navigation_;}

    /**
     * Returns pointer to the guidance module the vehicle uses. 
     * @returns Guidance_Interface
     */
    Guidance_Interface* getGuidanceModulePointer() {return guidance_;}

    /**
     * Returns pointer to the control module the vehicle uses. 
     * @returns Control_Interface
     */
    Control_Interface* getControlModulePointer() {return control_;}

    /**
     * Returns pointer to the dynamics module the vehicle uses. 
     * @returns Dynamics_Interface
     */
    Dynamics_Interface* getDynamicsModulePointer() {return dynamics_;}

    /**
     * Arms vehicle. Vehicle will ready itsself and then go into flight mode.
     */
    void armVehicle() {
        if(vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Error && vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Arm) vehicleData_.vehicleMode = eVehicleMode_t::eVehicleMode_Prepare;
    }

    /**
     * Disarms vehicle. Vehicle should shutdown immediately
     */
    void disarmVehicle() {
        if(vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Error) vehicleData_.vehicleMode = eVehicleMode_t::eVehicleMode_Disarm;
        
    }

    /**
     * Gets the vehicle data.
     * 
     * @returns VehicleData of vehicle
     */
    VehicleData getVehicleData() {return vehicleData_;}


protected:

    //Set to true when vehicle is ready for flight.
    bool vehicleInitialized_ = false;

    //Contains vehicle data.
    VehicleData vehicleData_;

    //Points to the navigation module to use.
    Navigation_Interface* navigation_;

    //Points to the guidance module to use.
    Guidance_Interface* guidance_;

    //Points to the control module to use.
    Control_Interface* control_;

    //Points to the dynamics module to use.
    Dynamics_Interface* dynamics_;


};



#endif