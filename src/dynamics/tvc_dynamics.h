#ifndef TVC_DYNAMICS_H
#define TVC_DYNAMICS_H



#include "Arduino.h"

#include "data_containers/dynamic_data.h"



class TVCDynamics {
public: 

    /**
     * tvcNeutralDirection is the direction of the force the TVC will produce 
     * when its angle is at 0.
     * 
     * tvcPosition is a vector<> pointing from vehicle CG to the point the TVC applies 
     * its force.
     *
     * @param values tvcPosition and tvcNeutralDirection.
     * @return none.
     */
    TVCDynamics (const Vector<> &tvcPosition, const Vector<> &tvcNeutralDirection) {
        _tvcPosition = tvcPosition;
        _tvcNeutralDirection = Quaternion<>(Vector<>(0,0,1).cross(tvcNeutralDirection), Vector<>(0,0,1).getAngleTo(tvcNeutralDirection)); //get rotation for nuetralDirection vector
    }

    /**
     * Updates the TVC parameters if they have changed.
     * 
     * tvcNeutralDirection is the direction of the force the TVC will produce 
     * when its angle is at 0.
     * 
     * tvcPosition is a vector<> pointing from vehicle CG to the point the TVC applies 
     * its force.
     *
     * @param values tvcPosition and tvcNeutralDirection.
     * @return none.
     */
    void setTVCParameters(const Vector<> &tvcPosition, const Vector<> &tvcNeutralDirection) {
        _tvcPosition = tvcPosition;
        _tvcNeutralDirection = Quaternion<>(Vector<>(0,0,1).cross(tvcNeutralDirection), Vector<>(0,0,1).getAngleTo(tvcNeutralDirection));
        _valid = false;
    }

    /**
     * Inputs the wanted dynamic setting.
     * 
     * if forceMaster is set to true then the force will always be correct 
     * and torqe will be resulting. 
     *
     * @param values Dynamic data.
     * @return none.
     */
    void dynamicsSetpoint(const DynamicData &setpoint, const bool &forceMaster = false) {_setpoint = setpoint; _forceMaster = forceMaster; _valid = false;}

    /**
     * Sets the constraints for dynamics.
     * 
     * maxForce gives the magnitude if the maximum force the TVC can produce.
     * maxAngle give the maximum gimble angle (in Radians) the TVC can go to.
     *
     * @param values maxForce and maxAngle.
     * @return none.
     */
    void setDynamicConstraints(float maxForce, float maxAngle) {
        _maxForce = maxForce;
        _maxAngle = maxAngle;
    }

    /**
     * Calculates the resulting dynamics the setpoint will have.
     *
     * @param values none.
     * @return Dynamic data.
     */
    DynamicData getResultingDynamics() {

        _calculate();

        DynamicData output;

        output.force = _tvcDirection*_tvcForce;
        output.torqe = _tvcPosition.cross(Fm);

        return output;

    }

    /**
     * Calculates the magnitude and force the TVC has to have.
     * 
     * Parameters are used as refernces meaning the will used as outputs.
     * 
     * tvcDirection is a unit vector<> and in reference to the neutral direction
     * vector.
     *
     * @param values tvcForce and tvcDirection.
     * @return none.
     */
    void getTVCSettings(float &tvcForce, Vector<> &tvcDirection) {

        _calculate();

        tvcForce = _tvcForce;
        tvcDirection = _tvcDirection;

    }


private:

    void _calculate() {

        if (_valid) return; //Data is still valid. Leave.

        if (_setpoint.torqe.isZeroVector()) {

            _tvcDirection = -_tvcPosition.copy().normalize(); //Rotate vector<> to gimble angle.

            _tvcForce = _setpoint.force^(-_tvcPosition.copy()).normalize(); //Project to get force.
            _tvcForce = constrain(_tvcForce, _maxForce, 0.0f); //Limit to force constraint.

        } else {

            Fm = _setpoint.torqe.cross(_tvcPosition.copy().normalize())/_tvcPosition.magnitude();
            Vector<> Fp = (_setpoint.force - Fm).getProjectionOn(_tvcPosition);

            float FmMag = Fm.magnitude();
            //Vector<> FmDir = Fm.copy().normalize();

            float FpMag = Fp.magnitude();

            float ang = atan2f(FmMag, FpMag); //Get angle of TVC.
            ang = min(ang, _maxAngle); //Limit angle to constraint.

            Quaternion<> rotation = Quaternion<>(_tvcPosition.cross(Fm), ang); //Get TVC rotation in body frame space
            rotation = _tvcNeutralDirection*rotation; //Get TVC rotation in body frame space

            _tvcDirection = (rotation*(_tvcPosition.copy().normalize())*rotation.copy().conjugate()).toVector(); //Rotate vector<> to gimble angle.

            if (_tvcDirection.isZeroVector()) _tvcDirection = Vector<>(0,0,1);

            if (FmMag == 0) {
                _tvcForce = 0;
            } else if (ang == 0) {
                _tvcForce = _maxForce;
            } else {
                _tvcForce = FmMag/sin(ang); //Project to get force.
                _tvcForce = constrain(_tvcForce, _maxForce, 0.0f); //Limit to force constraint.
            }

            _tvcDirection.z *= -1;
            

        }      

        

        _valid = true;

    }


    DynamicData _setpoint;

    float _maxForce = 0, _maxAngle = 0;

    Vector<> _tvcPosition;
    Quaternion<> _tvcNeutralDirection;

    //Is set to false when variables need to be recalculated.
    bool _valid = false;

    bool _forceMaster = false;

    Vector<> _tvcDirection;
    float _tvcForce = 0;

    Vector<> Fm;


    
};




#endif