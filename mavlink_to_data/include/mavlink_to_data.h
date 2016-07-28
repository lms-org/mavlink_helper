#ifndef MAVLINK_TO_DATA_H
#define MAVLINK_TO_DATA_H

#include <lms/module.h>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>
#include <street_environment/car.h>
#include <sensor_utils/sensor.h>

#include "sensors.h"

/**
 * @brief LMS module mavlink_to_data
 **/
class MavlinkToData : public lms::Module {
protected:
    enum class Timebase {
        FIXED,
        SYSTEM,
        MAVLINK
    };
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
    void configsChanged() override;

protected:
    // Sensor data accumulation
    typedef std::pair<uint8_t, uint8_t> SensorKey; // Message Identifier + Component ID
    typedef std::vector< std::shared_ptr<sensor_utils::Sensor> > SensorAccumulator; // Number of samples + accumulated data
    typedef std::map<SensorKey, SensorAccumulator> SensorAccumulatorContainer;

    template<typename T>
    using SensorConfig = std::map<size_t, T>;

protected:
    void parseIncomingMessages();
    void parseHeartBeat(const mavlink_message_t &msg);
    void parseIMU(const mavlink_message_t &msg);
    void parseOdometer(const mavlink_message_t &msg);
    void parseProximity(const mavlink_message_t &msg);

    // Configs
    const IMUConfig& getIMUConfig(size_t id, bool forceReload = false);
    const OdometerConfig& getOdometerConfig(size_t id, bool forceReload = false);
    const ProximityConfig& getProximityConfig(size_t id, bool forceReload = false);

    void accumulateMessages();
    void accumulateIMU(uint8_t sensorId, SensorAccumulator& samples);
    void accumulateOdometer(uint8_t sensorId, SensorAccumulator& samples);

    void computeCurrentTimestamp();


    // Datachannels
    lms::ReadDataChannel<Mavlink::Data> inChannel;
    //Datachannels filled with inChannel-data
    lms::WriteDataChannel<street_environment::Car> car;
    lms::WriteDataChannel<sensor_utils::SensorContainer> sensors;
    lms::WriteDataChannel<street_environment::Car::State> debugRcCarState;

    SensorAccumulatorContainer accumulator;

    //! internal sensor timestamp for fixed-timestep base
    Timebase timebase;
    lms::Time timestamp;
    lms::Time rawTimestamp;
    int heartBeatsMissed;

    // Sensor configs
    SensorConfig<IMUConfig> imuConfigs;
    SensorConfig<OdometerConfig> odometerConfigs;
    SensorConfig<ProximityConfig> proximityConfigs;
};

#endif // MAVLINK_TO_DATA_H
