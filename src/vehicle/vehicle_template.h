#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "definitions.h"

#include "data_containers/kinematic_data.h"
#include "data_containers/vehicle_mode.h"

#include "modules/guidance_modules/guidance_template.h"
#include "modules/navigation_modules/navigation_template.h"
#include "modules/control_modules/control_template.h"
#include "modules/dynamics_modules/dynamics_template.h"



class Vehicle {
public:

    /**
     * Thread function of the vehicle. 
     * All calculations the vehicle ever has to do for its 
     * control will be done here.
     *
     * @param values none.
     * @return none.
     */
    virtual void thread() = 0;

    /**
     * Init function that setups the vehicle. If not called
     * then on the first Thread run this will automatically 
     * be called.
     *
     * @param values none.
     * @return none.
     */
    virtual void init(Guidance* guidanceModule, Navigation* navigationModule, Control* controlModule, Dynamics* dynamicsModule) {

        _guidance = guidanceModule;
        _navigation = navigationModule;
        _control = controlModule;
        _dynamics = dynamicsModule; 

    };

    /**
     * Returns true if vehicle is ready to be armed
     *
     * @param values none.
     * @return bool.
     */
    virtual bool vehicleReady() = 0;

    /**
     * Returns how fast the module should be ran at.
     *
     * @param values none.
     * @return uint32_t.
     */
    virtual uint32_t getLoopRate_Hz() {
        return loopRate_Hz;
    }

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
    virtual DynamicData getControlData() {return _control->getDynamicsOutput();}

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

    //How fast the vehicle loop should be ran at
    uint32_t loopRate_Hz = 1000;


};



#endif