#ifndef DEVICE_STATUS_H
#define DEVICE_STATUS_H


enum DeviceStatus {
    DEVICE_NOT_STARTED,
    DEVICE_STARTING,
    DEVICE_CALIBRATING,
    DEVICE_RUNNING,
    DEVICE_RESTARTATTEMPT,
    DEVICE_FAILURE
};


inline String deviceStatusToString(DeviceStatus status) {

    String buff = "UNKNOWN STATE";

    switch (status)
    {
    case DeviceStatus::DEVICE_NOT_STARTED :
        buff = "Device not started";
        break;

    case DeviceStatus::DEVICE_STARTING :
        buff = "Device starting";
        break;

    case DeviceStatus::DEVICE_CALIBRATING :
        buff = "Device calibrating";
        break;

    case DeviceStatus::DEVICE_RUNNING :
        buff = "Device running";
        break;

    case DeviceStatus::DEVICE_RESTARTATTEMPT:
        buff = "Device attempting restart";
        break;

    case DeviceStatus::DEVICE_FAILURE :
        buff = "Device failure";
        break;
    
    default:
        buff = "UNKNOWN STATE: " + String(status);
        break;
    }

    return buff;

}





#endif