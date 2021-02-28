#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "data_containers/kinematic_data.h"

#include "modules/guidance_modules/guidance_template.h"
#include "modules/navigation_modules/navigation_template.h"
#include "modules/control_modules/control_template.h"
#include "modules/dynamics_modules/dynamics_template.h"
#include "modules/output_modules/output_template.h"



/**
 * Enum containing all vehicle modes.
 */
enum VEHICLE_MODE {
    //When this is set, the vehicle turns everything off due to a critical error in a subsystem. Should only be resetable from power reset.
    MODE_ERROR,
    //When this is set, the vehicle turns everything off. Should be resetable from vehicle.
    MODE_FAILSAFE,
    //When this is set, the vehicle prepares itsself to arm. Dangerous actuators like motors must be shutoff.
    MODE_DISARM,
    //When this is set, the vehicle turns everything on and follows flight commands.
    MODE_ARM
};


class Vehicle: public Module {
public:

    /**
     * Returns true if vehicle is ready for flight.
     *
     * @param values none.
     * @return bool.
     */
    virtual bool vehicleReady() = 0;

    /**
     * Returns the navigation data.
     *
     * @param values none.
     * @return navigation data.
     */
    virtual NavigationData getNavigationData() {return _navigation->getNavigationData();}

    /**
     * Returns the guidance output.
     *
     * @param values none.
     * @return controlData.
     */
    virtual ControlData getGuidanceData() {return _guidance->getControlSetpoint();}

    /**
     * Returns the control output.
     *
     * @param values none.
     * @return controlData.
     */
    virtual KinematicData getControlData() {return _control->getKinematicOutput();}

    /**
     * Returns a pointer to the navigation module.
     *
     * @param values none.
     * @return Navigation pointer.
     */
    virtual Navigation* getNavigationPointer() {return _navigation;}

    /**
     * Returns a pointer to the guidance module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual Guidance* getGuidancePointer() {return _guidance;}

    /**
     * Returns a pointer to the control module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual Control* getControlPointer() {return _control;}

    /**
     * Returns a pointer to the dynamics module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual Dynamics* getDynamicsPointer() {return _dynamics;}

    /**
     * Returns a pointer to the navigation module.
     *
     * @param values none.
     * @return Navigation pointer.
     */
    virtual void setNavigationPointer(Navigation* navigationModule) {_navigation = navigationModule;}

    /**
     * Returns a pointer to the guidance module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual void setGuidancePointer(Guidance* guidanceModule) {_guidance = guidanceModule;}

    /**
     * Returns a pointer to the control module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual void setControlPointer(Control* controlModule) {_control = controlModule;}

    /**
     * Returns a pointer to the dynamics module.
     *
     * @param values none.
     * @return Guidance pointer.
     */
    virtual void setDynamicsPointer(Dynamics* dynamicsModule) {_dynamics = dynamicsModule;}


protected:

    //Contains current forces exerted on the vehicle.
    static VEHICLE_MODE _vehicleMode;

    //Points to the navigation module to use.
    static Navigation *_navigation;

    //Points to the guidance module to use.
    static Guidance *_guidance;

    //Points to the control module to use.
    static Control *_control;

    //Points to the dynamics module to use.
    static Dynamics *_dynamics;


};


//Navigation Vehicle::_navigation = nullptr;




#endif