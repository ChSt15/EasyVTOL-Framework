#ifndef MANUAL_PROFILE_H
#define MANUAL_PROFILE_H



#include "flight_profiles/control_profile_template.h"

#include "vehicle/starship/starship.h"



class StarshipTestingProfile: public ControlProfile {
public:

    StarshipTestingProfile() {
        loopRate_Hz = 100;
    }

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();
    

    /**
     * Gives the vehicle control module the vehicle to control
     *
     * @param values vehiclePointer.
     * @return none.
     */
    void setVehiclePointer(Starship* vehiclePointer) {_vehicle = vehiclePointer;}

    /**
     * Returns vehicle being used by the control module
     *
     * @param values none.
     * @return vehiclePointer.
     */
    Starship* getVehiclePointer() {return _vehicle;}



private:

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    //Vehicle to be controlled
    Starship* _vehicle;

    //Extra control data container for linking and testing.
    ControlData _vehicleSetpoint;

    
};





#endif