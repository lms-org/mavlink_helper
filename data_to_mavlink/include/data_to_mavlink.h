#ifndef DATA_TO_MAVLINK_H
#define DATA_TO_MAVLINK_H

#include <lms/module.h>

#include <sensor_utils/car.h>
#include <sensor_utils/sensor.h>
#include "sensor_utils/distance_sensor.h"

#include <vector>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"

class CarToMavlink : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    void setControlCommands();
    
protected:
    // Datachannels
    lms::WriteDataChannel<Mavlink::Data> outChannel;
    lms::ReadDataChannel<sensor_utils::Car> car;

    
    // RC state
    uint8_t lastRcState;
};

#endif // DATA_TO_MAVLINK_H
