#ifndef CONTROL_PROFILE_TEMPLATE_H
#define CONTROL_PROFILE_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "vehicle/vehicle_template.h"



class ControlProfile: public Module {
public:

    /**
     * Gives the vehicle control module the vehicle to control
     *
     * @param values vehiclePointer.
     * @return none.
     */
    void setVehiclePointer(Vehicle* vehiclePointer) {_vehicle = vehiclePointer;}

    /**
     * Returns vehicle being used by the control module
     *
     * @param values none.
     * @return vehiclePointer.
     */
    Vehicle* getVehiclePointer() {return _vehicle;}

protected:

    //Vehicle to be controlled
    static Vehicle* _vehicle;

    
};





#endif