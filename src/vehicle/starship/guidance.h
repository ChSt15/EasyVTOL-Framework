#ifndef GUIDANCE_H
#define GUIDANCE_H



#include "Arduino.h"

#include "vehicle/guidance_template.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "circular_buffer.h"

#include "vehicle/kinetic_data.h"
#include "flight_settings.h"



class Guidance: GuidanceTemplate {
public:

    /**
     * Tells to vehicle to go to one point.
     * The path that the vehicle should take might not be 
     * a line. For that use toPointLinear().
     * This program also takes the velocities, and attitudes
     * of the start and end point.
     * Some vehicles might not support (ignore) certain
     * Kinetic parameters. E.g a multicopter due to it being 
     * underactuated will ignore attitude parameters. 
     * 
     * The time at arrival is the time at which the vehicle 
     * should reach the point relative to the last point or 
     * in the case that this has already happend then relative
     * to the time the function is called and is in microseconds.
     *
     * @param values point and time at arrival.
     * @return none.
     */
    void toPoint(KineticData state, uint32_t timeAtArrivalus);

    /**
     * Exacly the same as toPoint() (see for further infos) but 
     * interpolates the parameters to follow a line.
     *
     * @param values startPoint and endpoint.
     * @return none.
     */
    void toPointLinear(KineticData state, uint32_t timeAtArrivalus);

    /**
     * Tells to vehicle to go to a point that is relative
     * to the vehicle SETpoint not ISpoint. This also takes
     * the vehicles orientation into account. Meaning moving
     * 5 meters to the left is from the vehicles left.
     * The path that the vehicle should take might not be 
     * a line. For that use toPointLinearREL().
     * This program also takes the velocities, and attitudes
     * of the start and end point.
     * Some vehicles might not support (ignore) certain
     * Kinetic parameters. E.g a multicopter due to it being 
     * underactuated will ignore attitude parameters. 
     *
     * @param values only endpoint.
     * @return none.
     */
    void toPointREL(KineticData state, uint32_t timeAtArrivalus);

    /**
     * Exacly the same as toPointRel() (see for further infos) but 
     * interpolates the parameters to follow a line.
     *
     * @param values only endpoint.
     * @return none.
     */
    void toPointLinearREL(KineticData state, uint32_t timeAtArrivalus);

    /**
     * Returns the kinetic setpoints the vehicle has to follow
     * to achieve the commands.
     *
     * @param values none.
     * @return kinetic setpoint.
     */
    KineticData getGuidanceKineticSetpoint() {return _kineticSetpoint;}

    /**
     * Sets the mode of flight.
     *
     * @param values flight mode.
     * @return none.
     */
    void setFlightMode(const FLIGHT_MODE &flightMode) {*_flightMode = flightMode;}

    /**
     * Returns the current flight mode.
     *
     * @param values none.
     * @return flight mode.
     */
    FLIGHT_MODE getFlightMode() {return *_flightMode;};

    /**
     * Sets the flights profile.
     *
     * @param values flight mode.
     * @return none.
     */
    void setFlightProfile(const FLIGHT_PROFILE &flightProfile) {*_flightProfile = flightProfile;}

    /**
     * Returns the current flight profile.
     *
     * @param values none.
     * @return flight mode.
     */
    FLIGHT_PROFILE getFlightProfile() {return *_flightProfile;};

    /**
     * Returns number of path states are left to be reached. 
     * This is helpfull if you want to check if the vehicle is about to reach the 
     * last point and need to add the next point to the path.
     *
     * @param values none.
     * @return number of points left.
     */
    uint8_t numberPathStatesToGo() {return _path.capacity() - _path.size();};
    

protected:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void guidanceThread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void guidanceInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);

private:

    Circular_Buffer<KineticData, 100> _path; //We will have up to 3 "states" that build the of path the vehicle should follow.

    KineticData _kineticSetpoint;

    FLIGHT_MODE* _flightMode;
    FLIGHT_PROFILE* _flightProfile;

};





#endif