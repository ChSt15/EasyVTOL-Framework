#ifndef MAGNETOMETER_INTERFACE_H
#define MAGNETOMETER_INTERFACE_H



#include "lib/Math-Helper/src/3d_math.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



enum class eMagCalibStatus_t {
    eMagCalibStatus_NotCalibrated,
    eMagCalibStatus_Calibrating,
    eMagCalibStatus_Calibrated
};



class Magnetometer_Interface {
public:

    /**
     * @returns current calibration status
     */
    virtual eMagCalibStatus_t getCalibrationStatus() {return eMagCalibStatus_t::eMagCalibStatus_NotCalibrated;};

    /**
     * Starts calibration.
     */
    virtual void startCalibration() {};

    /**
     * Stops calibration sequence.
     */
    virtual void stopCalibration() {};

    /**
     * @returns reference to gyro data topic
     */
    const Topic<DataTimestamped<Vector<>>>& getMagTopic() const {return magTopic_;}


protected:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<Vector<>>> magTopic_;



    
};





#endif