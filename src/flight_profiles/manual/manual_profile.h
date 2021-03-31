#ifndef MANUAL_PROFILE_H
#define MANUAL_PROFILE_H



#include "flight_profiles/control_profile_template.h"

#include "modules/guidance_modules/guidance_flybywire.h"

#include "comms/ibus_rx.h"



class ManualControlProfile: public ControlProfile {
public:

    ManualControlProfile() {
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
    void setVehiclePointer(Vehicle* vehiclePointer) {_vehicle = vehiclePointer;}

    /**
     * Returns vehicle being used by the control module
     *
     * @param values none.
     * @return vehiclePointer.
     */
    Vehicle* getVehiclePointer() {return _vehicle;}



private:

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    //Guidance module given to vehicle to be used.
    GuidanceFlyByWire _guidanceFBW;

    //Vehicle to be controlled
    Vehicle* _vehicle;

    
};





#endif