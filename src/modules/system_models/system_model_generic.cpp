#include "../../../include/KraftKontrol/modules/system_models/system_model_generic.h"



SystemModelGeneric::SystemModelGeneric(float systemMass, VectorOLD<> systemAngularMass) {

    systemMass_ = systemMass;
    systemAngularMass_ = systemAngularMass;

}



void SystemModelGeneric::predictState(DataTimestamped<NavigationData>& systemState, int64_t time, uint32_t iterations) {


    //Time saving time calculations
    float dTime = float(time-systemState.timestamp)/SECONDS;

    float dTimePow2 = dTime*dTime;
    float dTimeHalfPow2 = dTimePow2*0.5f;
    float dTimeQuartPow4 = dTimePow2*dTimePow2*0.25f;


    //Collect info on actuators
    //DynamicData dynamicState = Actuator_Abstract::getSumDynamics();

    //Calculate resulting accelerations
    //systemState.data.angularAcceleration = systemState.data.attitude.rotateVector(dynamicState.torqe)/vehicleMomentInertia;
    //systemState.data.angularAccelerationError = systemState.data.angularAccelerationError + 10*dTime;

    //Predict angular stuff
    systemState.data.angularRate = systemState.data.angularRate + systemState.data.angularAcceleration*dTime; //Integrate angular accel to get new angular rate
    systemState.data.angularRateError = systemState.data.angularAccelerationError*dTime + systemState.data.angularRateError; //Calculate new error

    VectorOLD<> angleChange = systemState.data.angularAcceleration*dTimeHalfPow2 + systemState.data.angularRate*dTime;
    systemState.data.attitude = FML::Quaternion<>(angleChange, angleChange.magnitude())*systemState.data.attitude;

    //Calculate resulting accelerations
    //systemState.data.acceleration = GRAVITY_VECTOR + systemState.data.attitude.rotateVector(dynamicState.force)/vehicleMass;
    //systemState.data.accelerationError = systemState.data.accelerationError + 10*dTime;

    //Predict positional stuff
    systemState.data.velocity = systemState.data.velocity + systemState.data.linearAcceleration*dTime; //Integrate linear accel for velocity
    systemState.data.velocityError = systemState.data.accelerationError*dTime + systemState.data.velocityError; //Calculate new error

    systemState.data.position = systemState.data.position + systemState.data.velocity*dTime + systemState.data.linearAcceleration*dTimeHalfPow2; //Integrate linear accel for velocity
    systemState.data.positionError = systemState.data.accelerationError*dTimeHalfPow2 + systemState.data.velocityError*dTime + systemState.data.positionError; //Calculate new error

    systemState.data.absolutePosition.height = systemState.data.position.z + systemState.data.homePosition.height;

    //Update time
    systemState.timestamp = time;


}



float SystemModelGeneric::getSystemMass() const {
    return systemMass_;
}



void SystemModelGeneric::setSystemMass(float mass) {
    systemMass_ = mass;
}



VectorOLD<> SystemModelGeneric::getSystemAngularMass() const {
    return systemAngularMass_;
}



void SystemModelGeneric::setSystemAngularMass(const VectorOLD<>& angularMass) {
    systemAngularMass_ = angularMass;
}
