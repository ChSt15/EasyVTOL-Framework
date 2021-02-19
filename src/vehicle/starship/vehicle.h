#ifndef VEHICLE_H
#define VEHICLE_H



#include "definitions.h"

#include "vehicle/template_modules/vehicle_template.h"

#include "utils/interval_control.h"

#include "guidance.h"
#include "vehicle/standard_modules/navigation_complementary.h"
#include "control.h"
#include "dynamics.h"
#include "output_control.h"

#include "vehicle/flight_modes.h"


#define VEHICLE_LOOP_RATE 4000



class Vehicle: public VehicleTemplate, public Guidance, public NavigationComplementary, public Control, public Dynamics, public OutputControl {
public:

    /**
     * Thread function of the vehicle. 
     * All calculations the vehicle ever has to do for its 
     * control will be done here.
     *
     * @param values none.
     * @return none.
     */
    void vehicleThread();

    /**
     * Init function that setups the vehicle. If not called
     * then on the first Thread run this will automatically 
     * be called.
     *
     * @param values none.
     * @return none.
     */
    void vehicleInit();

    /**
     * Returns true if the vehicle is ready for flight.
     *
     * @param values none.
     * @return bool.
     */
    bool vehicleReady() {return _vehicleInitialized;}

    /**
     * Sets the vehicles flight mode;
     *
     * @param values flight mode.
     * @return none.
     */
    void setVehicleMode(const FLIGHT_MODE &flightMode) {_flightMode = flightMode;};

    /**
     * Sets the vehicles flight profile;
     *
     * @param values flight profile.
     * @return none.
     */
    void setVehicleProfile(const FLIGHT_PROFILE &flightProfile) {_flightProfile = flightProfile;};

    /**
     * Returns the vehicles flight mode.
     *
     * @param values none.
     * @return FLIGHT_MODE.
     */
    FLIGHT_MODE getVehicleMode() {return _flightMode;};

    /**
     * Returns the vehicles flight profile.
     *
     * @param values none.
     * @return FLIGHT_PROFILE.
     */
    FLIGHT_PROFILE getVehicleProfile() {return _flightProfile;};


private:

    //limits the vehicle loop rate
    IntervalControl interval = IntervalControl(VEHICLE_LOOP_RATE);

    bool _vehicleInitialized = false;

    FLIGHT_MODE _flightMode = FLIGHT_MODE::FAILSAFE;
    FLIGHT_PROFILE _flightProfile = FLIGHT_PROFILE::HOVER;

};





#endif