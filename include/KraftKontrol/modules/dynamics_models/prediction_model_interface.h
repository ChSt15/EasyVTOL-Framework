#ifndef PREDICTION_MODEL_INTERFACE_H
#define PREDICTION_MODEL_INTERFACE_H



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"



class Prediction_Model_Interface {
public:

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @returns KinematicData struct of predicted state.
     */
    virtual KinematicData getPrediction() = 0;

    
    
    
};





#endif