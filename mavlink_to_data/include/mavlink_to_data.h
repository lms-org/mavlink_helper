#ifndef MAVLINK_TO_DATA_H
#define MAVLINK_TO_DATA_H

#include <lms/module.h>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"
#include <sensor_utils/car.h>
#include <sensor_utils/sensor.h>
#include <sensor_utils/odometer.h>
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
    // Sensor data accumulation
    typedef std::pair<uint8_t, uint8_t> SensorKey; // Message Identifier + Component ID
    typedef std::vector< std::shared_ptr<sensor_utils::Sensor> > SensorAccumulator; // Number of samples + accumulated data
    typedef std::map<SensorKey, SensorAccumulator> SensorAccumulatorContainer;

protected:
    void parseIncomingMessages();
    void parseHeartBeat(const mavlink_message_t &msg);
    void parseIMU(const mavlink_message_t &msg);
    void parseOdometer(const mavlink_message_t &msg);
    void parseProximity(const mavlink_message_t &msg);

    void accumulateMessages();
    void accumulateIMU(uint8_t sensorId, SensorAccumulator& samples);
    void accumulateOdometer(uint8_t sensorId, SensorAccumulator& samples);

    // Datachannels
    lms::ReadDataChannel<Mavlink::Data> inChannel;
    //Datachannels filled with inChannel-data
    lms::WriteDataChannel<sensor_utils::Car> car;
    lms::WriteDataChannel<sensor_utils::SensorContainer> sensors;

    SensorAccumulatorContainer accumulator;
};

#endif // MAVLINK_TO_DATA_H
