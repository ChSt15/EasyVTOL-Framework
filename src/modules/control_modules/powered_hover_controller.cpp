#include "powered_hover_controller.h"




void HoverController::thread() {

    
    //Attitude control section

    if (_controlSetpoint->attitudeControlMode == CONTROL_MODE::CONTROL_DISABLED) {

        _controlOutput.force = 0;
        _controlOutput.torqe = 0;

    } else {    

        _controlOutput.torqe = 0;

        ControlData setpoint = *_controlSetpoint;

        Vector attitudeOutput(0);
        Vector angVelOutput(0);
        Vector angAccelOutput(0);

        Vector outputTotal(0);

        if (setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_POSITION || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY_POSITION || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION) {

            Vector error = (setpoint.attitude*_navigationData->attitude.copy().conjugate()).toVector(); //Error is calculated here already in local coordinate system.

            _attitudeIValue += error.compWiseMulti(_attitudeIF);

            Vector attitudeOutput = error.compWiseMulti(_attitudePF) + (setpoint.angularRate - _navigationData->angularRate).compWiseMulti(_attitudeDF) + _attitudeIValue;

            if (attitudeOutput.x > _attitudeLimit.x) {
                _attitudeIValue.x -= attitudeOutput.x - _attitudeLimit.x; //Remove saturation from I according to overthreshold
                _attitudeIValue.x = max(_attitudeIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.x < -_attitudeLimit.x) {
                _attitudeIValue.x -= attitudeOutput.x + _attitudeLimit.x; //Remove saturation from I according to overthreshold
                _attitudeIValue.x = min(_attitudeIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (attitudeOutput.y > _attitudeLimit.y) {
                _attitudeIValue.y -= attitudeOutput.y - _attitudeLimit.y; //Remove saturation from I according to overthreshold
                _attitudeIValue.y = max(_attitudeIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.y < -_attitudeLimit.y) {
                _attitudeIValue.y -= attitudeOutput.y + _attitudeLimit.y; //Remove saturation from I according to overthreshold
                _attitudeIValue.y = min(_attitudeIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (attitudeOutput.z > _attitudeLimit.z) {
                _attitudeIValue.z -= attitudeOutput.z - _attitudeLimit.z; //Remove saturation from I according to overthreshold
                _attitudeIValue.z = max(_attitudeIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.z < -_attitudeLimit.z) {
                _attitudeIValue.z -= attitudeOutput.z + _attitudeLimit.z; //Remove saturation from I according to overthreshold
                _attitudeIValue.z = min(_attitudeIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            attitudeOutput = error.compWiseMulti(_attitudePF) + (setpoint.angularRate - _navigationData->angularRate).compWiseMulti(_attitudeDF) + _attitudeIValue; //Recalculate new output

            //Constrain output
            attitudeOutput.x = constrain(attitudeOutput.x, -_attitudeLimit.x, _attitudeLimit.x);
            attitudeOutput.y = constrain(attitudeOutput.y, -_attitudeLimit.y, _attitudeLimit.y);
            attitudeOutput.z = constrain(attitudeOutput.z, -_attitudeLimit.z, _attitudeLimit.z);

            if (_attitudePassThrough) outputTotal += attitudeOutput;
            else setpoint.angularRate += attitudeOutput;

        }

        if (setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_VELOCITY_POSITION || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION) {

            Vector error = _navigationData->attitude.copy().conjugate().rotateVector(setpoint.angularRate - _navigationData->angularRate); //Calculate setpoint error and then rotate to local coordinate system.

            _angVelIValue += error.compWiseMulti(_angVelIF);

            Vector angVelOutput = error.compWiseMulti(_angVelPF) + (setpoint.angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) + _angVelIValue;

            if (angVelOutput.x > _angVelLimit.x) {
                _angVelIValue.x -= angVelOutput.x - _angVelLimit.x; //Remove saturation from I according to overthreshold
                _angVelIValue.x = max(_angVelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.x < -_angVelLimit.x) {
                _angVelIValue.x -= angVelOutput.x + _angVelLimit.x; //Remove saturation from I according to overthreshold
                _angVelIValue.x = min(_angVelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.y > _angVelLimit.y) {
                _angVelIValue.y -= angVelOutput.y - _angVelLimit.y; //Remove saturation from I according to overthreshold
                _angVelIValue.y = max(_angVelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.y < -_angVelLimit.y) {
                _angVelIValue.y -= angVelOutput.y + _angVelLimit.y; //Remove saturation from I according to overthreshold
                _angVelIValue.y = min(_angVelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.z > _angVelLimit.z) {
                _angVelIValue.z -= angVelOutput.z - _angVelLimit.z; //Remove saturation from I according to overthreshold
                _angVelIValue.z = max(_angVelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.z < -_angVelLimit.z) {
                _angVelIValue.z -= angVelOutput.z + _angVelLimit.z; //Remove saturation from I according to overthreshold
                _angVelIValue.z = min(_angVelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angVelOutput = error.compWiseMulti(_angVelPF) + (setpoint.angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) + _angVelIValue; //Recalculate new output

            //Constrain output
            angVelOutput.x = constrain(angVelOutput.x, -_angVelLimit.x, _angVelLimit.x);
            angVelOutput.y = constrain(angVelOutput.y, -_angVelLimit.y, _angVelLimit.y);
            angVelOutput.z = constrain(angVelOutput.z, -_angVelLimit.z, _angVelLimit.z);

            if (_angVelPassThrough) outputTotal += angVelOutput;
            else setpoint.angularAcceleration += angVelOutput;
            

        }

        if (setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY || setpoint.attitudeControlMode == CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION) {

            Vector error = _navigationData->attitude.copy().conjugate().rotateVector(setpoint.angularAcceleration - _navigationData->angularAcceleration); //Calculate setpoint error and then rotate to local coordinate system.

            _angAccelIValue += error.compWiseMulti(_angAccelIF);

            Vector angAccelOutput = error.compWiseMulti(_angAccelPF)/* + (setpoint.angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) currently not implemented */ + _angAccelIValue;

            if (angAccelOutput.x > _angAccelLimit.x) {
                _angAccelIValue.x -= angAccelOutput.x - _angAccelLimit.x; //Remove saturation from I according to overthreshold
                _angAccelIValue.x = max(_angAccelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.x < -_angAccelLimit.x) {
                _angAccelIValue.x -= angAccelOutput.x + _angAccelLimit.x; //Remove saturation from I according to overthreshold
                _angAccelIValue.x = min(_angAccelIValue.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.y > _angAccelLimit.y) {
                _angAccelIValue.y -= angAccelOutput.y - _angAccelLimit.y; //Remove saturation from I according to overthreshold
                _angAccelIValue.y = max(_angAccelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.y < -_angAccelLimit.y) {
                _angAccelIValue.y -= angAccelOutput.y + _angAccelLimit.y; //Remove saturation from I according to overthreshold
                _angAccelIValue.y = min(_angAccelIValue.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.z > _angAccelLimit.z) {
                _angAccelIValue.z -= angAccelOutput.z - _angAccelLimit.z; //Remove saturation from I according to overthreshold
                _angAccelIValue.z = max(_angAccelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.z < -_angAccelLimit.z) {
                _angAccelIValue.z -= angAccelOutput.z + _angAccelLimit.z; //Remove saturation from I according to overthreshold
                _angAccelIValue.z = min(_angAccelIValue.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angAccelOutput = error.compWiseMulti(_angAccelPF)/* + (setpoint.angularAcceleration - _navigationData->angularAcceleration).compWiseMulti(_angVelDF) currently not implemented */ + _angAccelIValue; //Recalculate new output

            //Constrain output
            angAccelOutput.x = constrain(angAccelOutput.x, -_angAccelLimit.x, _angAccelLimit.x);
            angAccelOutput.y = constrain(angAccelOutput.y, -_angAccelLimit.y, _angAccelLimit.y);
            angAccelOutput.z = constrain(angAccelOutput.z, -_angAccelLimit.z, _angAccelLimit.z);

        }

        _controlOutput.torqe = outputTotal;

    }

    _controlOutput.force = Vector(0,0,9.81)*1; //Multiplied by vehicle mass

}



void HoverController::init() {

}
