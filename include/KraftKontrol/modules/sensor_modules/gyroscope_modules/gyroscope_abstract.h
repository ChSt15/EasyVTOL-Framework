#ifndef GYROSCOPE_ABSTRACT_H
#define GYROSCOPE_ABSTRACT_H



#include "lib/MathHelperLibrary/FML.h"

#include "../../module_abstract.h"
#include "../../../utils/topic_subscribers.h"
#include "../../../KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"
#include "../../communication_modules/kraft_message_subscriber.h"

#include "../sensordata.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/data_timestamped.h"




class Gyroscope_Abstract {
private:

    //Topic for distributing new measurements.
    Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>> valueTopic_;

    CommandMessageGyroCalValues calValues_;
    CommandMessageGyroMountTransform mountValues_;

    KraftMessage_Subscriber messageSubr_;


protected:

    Gyroscope_Abstract();


public:

    /**
     * @returns reference to gyro data topic
     */
    const Topic<DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>>>& getGyroTopic() const;

    /**
     * Sets the gyroscope axis scaling and aligning calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisScaleAlign Matrix to scale and align gyroscope axis.
     */
    void setGyroCalibrationAxisScaleAlign(const FML::Matrix33_F& axisScaleAlign);

    /**
     * @returns axisScaleAlign matrix used for calibration.
     */
    const FML::Matrix33_F& getGyroCalibrationAxisScaleAlign() const;
    
    /**
     * Sets the gyroscope bias calibration. 
     * Calibration formula: a' = S*(a + b) 
     * With:    a' calibrated value
     *          a raw value
     *          S  scale and alignment
     *          b  bias 
     * @param axisBias Vector bias for each axis.
     */
    void setGyroCalibrationAxisBias(const FML::Vector3_F& axisBias);

    /**
     * @returns axisBias vector used for calibration.
     */
    const FML::Vector3_F& getGyroCalibrationAxisBias() const;

    /**
     * Sets the gyroscope transformation matrix.
     * Transformation formula: a' = Sa
     * With:    a' transformed value
     *          a  calibrated value
     *          S  transformation matrix
     * @param mountTransform Matrix to transform the data into local frame coordinates.
     */
    void setGyroTransform(const FML::Matrix33_F& mountTransform);

    /**
     * @returns transformation matrix used to transform values into local frame.
     */
    const FML::Matrix33_F& getGyroTransform() const;

    /**
     * Transforms gyroscope data and then publishes it to topic.
     * @param gyroValues
     * @param calibrate If true then the values will be calibrated.
     * @param transform If true then the values will be transformed to local frame.
     */
    void publishGyroData(DataTimestamped<SensorData<FML::Vector3_F, FML::Matrix33_F>> gyroValues, bool calibrate = true, bool transform = true);
    
};





#endif