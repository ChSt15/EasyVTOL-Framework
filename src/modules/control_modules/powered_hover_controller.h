#ifndef LINEAR_PID_CONTROLLER_H
#define LINEAR_PID_CONTROLLER_H



#include "modules/templates/module_template.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"

#include "modules/control_modules/control_template.h"


class HoverController: public Control {
public:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setAngularAccelerationPIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_angAccelPF = factorP; _angAccelIF = factorI; _angAccelDF = factorD; _angAccelLimit = limit;}

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setAngularVelocityPIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_angVelPF = factorP; _angVelIF = factorI; _angVelDF = factorD; _angVelLimit = limit;}

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setAttitudePIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_attitudePF = factorP; _attitudeIF = factorI; _attitudeDF = factorD; _attitudeLimit = limit;}

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setAccelerationPIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_accelPF = factorP; _accelIF = factorI; _accelDF = factorD; _accelLimit = limit;}

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setVelocityPIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_velocityPF = factorP; _velocityIF = factorI; _velocityDF = factorD; _velocityLimit = limit;}

    /**
     * Sets the control factor.
     * 
     * Limit is the value that if reached, the intergrator will stop integrating and also reduce for anti-windup
     *
     * @param values factor.
     * @return none.
     */
    void setPositionPIDFactors(const Vector &factorP = 0, const Vector &factorI = 0, const Vector &factorD = 0, const Vector &limit = 1) {_positionPF = factorP; _positionIF = factorI; _positionDF = factorD; _positionLimit = limit;}


private:

    //P factor for angular acceleration 
    Vector _angAccelPF = 0;
    //I factor for angular acceleration 
    Vector _angAccelIF = 0;
    //D factor for angular acceleration 
    Vector _angAccelDF = 0;
    //I limit for angular acceleration 
    Vector _angAccelLimit = 1;
    //I value for angular acceleration
    Vector _angAccelIValue = 0; 

    //P factor for angular velocity 
    Vector _angVelPF = 0;
    //I factor for angular velocity 
    Vector _angVelIF = 0;
    //D factor for angular velocity 
    Vector _angVelDF = 0;
    //I limit for angular velocity 
    Vector _angVelLimit = 1;
    //I value for angular velocity
    Vector _angVelIValue = 0; 

    //P factor for attitude 
    Vector _attitudePF = 0;
    //I factor for attitude 
    Vector _attitudeIF = 0;
    //D factor for attitude  
    Vector _attitudeDF = 0;
    //I limit for attitude
    Vector _attitudeLimit = 1;
    //I value for attitude
    Vector _attitudeIValue = 0; 

    //P factor for acceleration 
    Vector _accelPF = 0;
    //I factor for acceleration 
    Vector _accelIF = 0;
    //D factor for acceleration  
    Vector _accelDF = 0;
    //I limit for acceleration
    Vector _accelLimit = 1;
    //I value for acceleration
    Vector _accelIValue = 0; 

    //P factor for velocity 
    Vector _velocityPF = 0;
    //I factor for velocity 
    Vector _velocityIF = 0;
    //D factor for velocity  
    Vector _velocityDF = 0;
    //I limit for velocity
    Vector _velocityLimit = 1;
    //I value for velocity
    Vector _velocityIValue = 0; 

    //P factor for position 
    Vector _positionPF = 0;
    //I factor for position 
    Vector _positionIF = 0;
    //D factor for position  
    Vector _positionDF = 0;
    //I limit for position
    Vector _positionLimit = 1;
    //I value for position
    Vector _positionIValue = 0; 


    
};





#endif