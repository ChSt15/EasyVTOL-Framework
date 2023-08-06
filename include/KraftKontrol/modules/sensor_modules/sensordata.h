#ifndef SENSORDATA_H
#define SENSORDATA_H


/**
 * Simple container class for storing sensor values and their covariances for distribution via topics.
 */
template<typename VALUETYPE, typename COVARIANCETYPE>
class SensorData {
public:

    ///Values of sensor data.
    VALUETYPE values;

    ///Covariance of sensor values.
    COVARIANCETYPE covariance;


    SensorData();

    SensorData(const VALUETYPE& values);

    SensorData(const VALUETYPE& values, const COVARIANCETYPE& covariance);
    
    
};



template<typename VALUETYPE, typename COVARIANCETYPE>
SensorData<VALUETYPE, COVARIANCETYPE>::SensorData() {}


template<typename VALUETYPE, typename COVARIANCETYPE>
SensorData<VALUETYPE, COVARIANCETYPE>::SensorData(const VALUETYPE& values) {

    this->values = values;

}


template<typename VALUETYPE, typename COVARIANCETYPE>
SensorData<VALUETYPE, COVARIANCETYPE>::SensorData(const VALUETYPE& values, const COVARIANCETYPE& covariance) {

    this->values = values;
    this->covariance = covariance;

}



#endif