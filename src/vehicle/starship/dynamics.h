#ifndef DYNAMICS_H
#define DYNAMICS_H


/**
 * This is where the vehicle "output mapping" is done. This should for example 
 * compensate for actuator errors from servos.
 * This allows the control to be written in a very general way and streamlines the 
 * development if the control systems. 
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
 * 
 * Dynamics does not require to be a closed loop solution. Meaning it could simply map the control
 * outputs to the vehicle controls (flaps, motors etc.) and the control algorithms can to the rest
 * but implementing closed loop control allows for more accuracy and further linearity of control 
 * outputs and simplifies the control algorithms. 
*/


#include "Arduino.h"

#include "vehicle/dynamics_template.h"

#include "output_control.h"

#include "vehicle/kinetic_data.h"
#include "vehicle/dynamic_data.h"

#include "flight_settings.h"



class Dynamics: DynamicsTemplate {
public:



protected:

    void dynamicsThread(KineticData vehicleKinetics);
    void dynamicsInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);
    
    /**
     * Sets the dynamic data. This should all be in local coordinate system
     *
     * @param values dynamics.
     * @return none.
     */
    void setDynamicsSetpoint(DynamicData dynamics) {_dynamicData = dynamics;}

    /**
     * Returns the actuator positions.
     * Definitions:
     * 
     * TVCServoXP: TVC servo that controls the flap in X axis positive. Range is -1 to 1.
     * TVCServoXN: TVC servo that controls the flap in X axis negative. Range is -1 to 1.
     * TVCServoYP: TVC servo that controls the flap in Y axis positive. Range is -1 to 1.
     * TVCServoYN: TVC servo that controls the flap in Y axis negative. Range is -1 to 1.
     * 
     * motorCW: Motor control for clockwise motor. Range is 0 to 1.
     * motorCCW: Motor control for counter clockwise motor. Range is 0 to 1.
     * 
     * flapTL: Servo that controls the drag flap on top left. Range is 0 to 1.
     * flapTR: Servo that controls the drag flap on top right. Range is 0 to 1.
     * flapBL: Servo that controls the drag flap on bottom left. Range is 0 to 1.
     * flapBR: Servo that controls the drag flap on bottom right. Range is 0 to 1.
     * 
     *
     * @param values lots but all will be changed by function as these are adresses.
     * @return none.
     */
    Outputs getActuatorSetpoints() {return _outputs;};
    //void getActuatorSetpoints(float &TVCServoXP, float &TVCServoXN, float &TVCServoYP, float &TVCServoYN, float &motorCW, float &motorCCW, float &flapTL, float &flapTR, float &flapBL, float &flapBR);
    

private:

    Outputs _outputs;

    DynamicData _dynamicData;

    FLIGHT_MODE* _flightMode;
    FLIGHT_PROFILE* _flightProfile;
    
    
};





#endif