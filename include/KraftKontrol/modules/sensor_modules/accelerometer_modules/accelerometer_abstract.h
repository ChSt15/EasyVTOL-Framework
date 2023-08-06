#ifndef ACCELEROMETER_ABSTRACT_H
#define ACCELEROMETER_ABSTRACT_H



#include "lib/MathHelperLibrary/FML.h"

#include "../../module_abstract.h"
#include "../../../utils/topic_subscribers.h"
#include "../../../KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"
#include "../../communication_modules/kraft_message_subscriber.h"

#include "../sensordata.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"



class Accelerometer_Abstract {
private:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> valueTopic_;

    CommandMessageAccelCalValues calValues_;
    CommandMessageAccelMountTransform mountValues_;

    KraftMessage_Subscriber messageSubr_;


protected:

    Accelerometer_Abstract();


public:

    /**
     * @returns reference to accel data topic
     */
    const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& getAccelTopic() const;

    /**
     * Sets the accelerometer axis scaling and aligning calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisScaleAlign Matrix to scale and align accelerometer axis.
     */
    void setAccelCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign);

    /**
     * @returns axisScaleAlign matrix used for calibration.
     */
    const FML::Matrix33_F& getAccelCalibrationAxisScaleAlign() const;
    
    /**
     * Sets the accelerometer bias calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisBias Vector bias for each axis.
     */
    void setAccelCalibrationAxisBias(const FML::Vector3_F& axisBias);

    /**
     * @returns axisBias vector used for calibration.
     */
    const FML::Vector3_F& getAccelCalibrationAxisBias() const;

    /**
     * Sets the accelerometer transformation matrix.
     * Transformation formula: a' = Sa
     * With:    a' transformed value
     *          a  calibrated value
     *          S  transformation matrix
     * @param mountTransform Matrix to transform the data into local frame coordinates.
     */
    void setAccelTransform(const FML::Matrix33_F& mountTransform);

    /**
     * @returns transformation matrix used to transform values into local frame.
     */
    const FML::Matrix33_F& getAccelTransform() const;

    /**
     * Transforms accelerometer data and then publishes it to topic.
     * @param accelValues
     * @param calibrate If true then the values will be calibrated.
     * @param transform If true then the values will be transformed to local frame.
     */
    void publishAccelData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> accelValues, bool calibrate = true, bool transform = true);
    
    
};





#endif