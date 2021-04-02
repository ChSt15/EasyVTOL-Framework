#include "belly_flop_control.h"




void SimplePIDController::thread() {

    
    //Attitude control section

    if (_controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_DISABLED) {

        _controlOutput.force = 0;
        _controlOutput.torqe = 0;

    } else {    

        _controlOutput.torqe = 0;

        if (_controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_POSITION || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY_POSITION || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY_POSITION) {

            Vector error = (_controlSetpoint->attitude*_navigationData->attitude.copy().conjugate()).toVector();

            _attitudeIValue += error.compWiseMulti(_attitudeIF);

            Vector output = error.compWiseMulti(_attitudePF) + (_controlSetpoint->angularRate - _navigationData->angularRate).compWiseMulti(_attitudeDF) + _attitudeIValue;

            if (output.x > _attitudeLimit.x) {
                _attitudeIValue.x -= output.x - _attitudeLimit.x; //Remove saturation from I according to overthreshold
                _attitudeIValue.x = max(_attitudeIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.y > _attitudeLimit.y) {
                _attitudeIValue.y -= output.y - _attitudeLimit.y; //Remove saturation from I according to overthreshold
                _attitudeIValue.y = max(_attitudeIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.z > _attitudeLimit.z) {
                _attitudeIValue.z -= output.z - _attitudeLimit.z; //Remove saturation from I according to overthreshold
                _attitudeIValue.z = max(_attitudeIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            output = error.compWiseMulti(_attitudePF) + (_controlSetpoint->angularRate - _navigationData->angularRate).compWiseMulti(_attitudeDF) + _attitudeIValue; //Recalculate new output

            //Constrain output
            output.x = constrain(output.x, -_attitudeLimit.x, _attitudeLimit.x);
            output.y = constrain(output.y, -_attitudeLimit.y, _attitudeLimit.y);
            output.z = constrain(output.z, -_attitudeLimit.z, _attitudeLimit.z);

            _controlOutput.torqe += output;

        }

        if (_controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY_POSITION || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION) {

            Vector error = _controlSetpoint->angularRate - _navigationData->angularRate;

            _angVelIValue += error.compWiseMulti(_angVelIF);

            Vector output = error.compWiseMulti(_angVelPF) + (_controlSetpoint->angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) + _angVelIValue;

            if (output.x > _angVelLimit.x) {
                _angVelIValue.x -= output.x - _angVelLimit.x; //Remove saturation from I according to overthreshold
                _angVelIValue.x = max(_angVelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.y > _angVelLimit.y) {
                _angVelIValue.y -= output.y - _angVelLimit.y; //Remove saturation from I according to overthreshold
                _angVelIValue.y = max(_angVelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.z > _angVelLimit.z) {
                _angVelIValue.z -= output.z - _angVelLimit.z; //Remove saturation from I according to overthreshold
                _angVelIValue.z = max(_angVelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            output = error.compWiseMulti(_angVelPF) + (_controlSetpoint->angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) + _angVelIValue; //Recalculate new output

            //Constrain output
            output.x = constrain(output.x, -_angVelLimit.x, _angVelLimit.x);
            output.y = constrain(output.y, -_angVelLimit.y, _angVelLimit.y);
            output.z = constrain(output.z, -_angVelLimit.z, _angVelLimit.z);

            _controlOutput.torqe += output;

        }

        if (_controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY || _controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION) {

            Vector error = _controlSetpoint->angularAcceleration - _navigationData->angularAcceleration;

            _angAccelIValue += error.compWiseMulti(_angAccelIF);

            Vector output = error.compWiseMulti(_angAccelPF)/* + (_controlSetpoint->angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) currently not implemented */ + _angAccelIValue;

            if (output.x > _angAccelLimit.x) {
                _angAccelIValue.x -= output.x - _angAccelLimit.x; //Remove saturation from I according to overthreshold
                _angAccelIValue.x = max(_angAccelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.y > _angAccelLimit.y) {
                _angAccelIValue.y -= output.y - _angAccelLimit.y; //Remove saturation from I according to overthreshold
                _angAccelIValue.y = max(_angAccelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (output.z > _angAccelLimit.z) {
                _angAccelIValue.z -= output.z - _angAccelLimit.z; //Remove saturation from I according to overthreshold
                _angAccelIValue.z = max(_angAccelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            output = error.compWiseMulti(_angAccelPF)/* + (_controlSetpoint->angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) currently not implemented */ + _angAccelIValue; //Recalculate new output

            //Constrain output
            output.x = constrain(output.x, -_angAccelLimit.x, _angAccelLimit.x);
            output.y = constrain(output.y, -_angAccelLimit.y, _angAccelLimit.y);
            output.z = constrain(output.z, -_angAccelLimit.z, _angAccelLimit.z);

            _controlOutput.torqe += output;

        }

    }

}



void SimplePIDController::init() {

}
