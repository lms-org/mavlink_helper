#ifndef MAVLINK_TO_DATA_H
#define MAVLINK_TO_DATA_H

#include <lms/module.h>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"
#include <sensor_utils/car.h>
#include <sensor_utils/sensor.h>
#include "sensor_utils/distance_sensor.h"
/**
 * @brief LMS module mavlink_to_data
 **/
class MavlinkToData : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;

protected:
    void parseIncomingMessages();
    void parseHeartBeat(const mavlink_message_t &msg);
    void parseIMU(const mavlink_message_t &msg);
    void parseOdometer(const mavlink_message_t &msg);
    void parseProximity(const mavlink_message_t &msg);

    // Datachannels
    lms::ReadDataChannel<Mavlink::Data> inChannel;
    //Datachannels filled with inChannel-data
    lms::WriteDataChannel<sensor_utils::Car> car;
    lms::WriteDataChannel<sensor_utils::SensorContainer> sensors;
};

#endif // MAVLINK_TO_DATA_H
