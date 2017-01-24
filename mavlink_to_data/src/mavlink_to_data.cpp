#include "mavlink_to_data.h"
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"
#include "sensor_utils/odometer.h"
#include "sensor_utils/parking_sensor.h"

#include <phoenix_CC2016_service/phoenix_CC2016_service.h>
#include <timestamp_interpolator_service/timestamp_interpolator_service.h>
#include <sensors.h>

bool MavlinkToData::initialize() {
    inChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    sensors = writeChannel<sensor_utils::SensorContainer>("SENSORS");
    debugRcCarState = writeChannel<street_environment::CarCommand::State>("RC_CAR_STATE");
    heartBeatsMissed = 0;

    // Configure sensor timebase
    auto tb = config().get<std::string>("sensor_timebase", "mavlink");
    if(tb == "fixed")
    {
        timebase = Timebase::FIXED;
    }
    else if(tb == "system")
    {
        timebase = Timebase::SYSTEM;
    }
    else if(tb == "mavlink")
    {
        timebase = Timebase::MAVLINK;
    }
    else
    {
        logger.error("computeCurrentTimestamp") << "Invalid sensor timebase";
        return false;
    }
    return true;
}

bool MavlinkToData::deinitialize() {
    return true;
}

bool MavlinkToData::cycle() {
    computeCurrentTimestamp();
    parseIncomingMessages();
    accumulateMessages();
    return true;
}

void MavlinkToData::parseIncomingMessages(){
    int heartBeatMsgs = 0;
    for( const auto& msg : *inChannel ){
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
            parseHeartBeat(msg);
            heartBeatMsgs++;
        }else if(msg.msgid == MAVLINK_MSG_ID_IMU){
            parseIMU(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER){
            parseOdometer(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA){
            parseOdometerDelta(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_PROXIMITY){
            parseProximity(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_PARKING_LOT){
            parseParking(msg);
        }
    }
    if(heartBeatMsgs == 0){
        heartBeatsMissed--;
        logger.info("parseIncomingMessages")<<"heart didn't beat"<<heartBeatsMissed;
    }else{
        heartBeatsMissed = 0;
        if(heartBeatMsgs > 1){
            logger.info("Got more than one heartbeat")<<heartBeatMsgs;
        }
    }
}

void MavlinkToData::accumulateMessages()
{
    for( auto& data : accumulator )
    {
        switch( data.first.first )
        {
            case MAVLINK_MSG_ID_IMU:
                accumulateIMU(data.first.second, data.second);
                break;
            case MAVLINK_MSG_ID_ODOMETER_DELTA | MAVLINK_MSG_ID_ODOMETER:
                accumulateOdometer(data.first.second, data.second);
                break;
        }
    }
}

void MavlinkToData::parseIMU(const mavlink_message_t &msg){
    mavlink_imu_t data;
    mavlink_msg_imu_decode(&msg,&data);

    std::shared_ptr<sensor_utils::IMU> imu = std::make_shared<sensor_utils::IMU>();

    imu->sensorId(msg.compid);

    // Config
    const auto& cfg = getIMUConfig(imu->sensorId(), false);
    imu->name(cfg.name);
    imu->timestamp(timestamp);

    // Measurements
    imu->accelerometer = sensor_utils::IMU::Measurement(data.xacc, data.yacc, data.zacc) - cfg.accBias;
    imu->gyroscope     = sensor_utils::IMU::Measurement(data.xgyro, data.ygyro, data.zgyro) - cfg.gyroBias;
    imu->magnetometer  = sensor_utils::IMU::Measurement(data.xmag, data.ymag, data.zmag);

    // Save message in accumulator
    auto accumulatorKey = std::make_pair( static_cast<uint8_t>(MAVLINK_MSG_ID_IMU), static_cast<uint8_t>(imu->sensorId()) );
    accumulator[accumulatorKey].push_back(imu);
}

void MavlinkToData::parseOdometer(const mavlink_message_t &msg){
    mavlink_odometer_t data;
    mavlink_msg_odometer_decode(&msg,&data);

    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();

    odometer->sensorId(msg.compid);
    const auto& cfg = getOdometerConfig(odometer->sensorId(), false);
    odometer->name(cfg.name);
    odometer->timestamp(timestamp);

    odometer->distance = sensor_utils::Odometer::Measurement( data.xdist_delta, data.ydist_delta, data.zdist_delta );
    odometer->absDistance = sensor_utils::Odometer::Measurement( data.xdist_abs, data.ydist_abs, data.zdist_abs );
    odometer->velocity = sensor_utils::Odometer::Measurement( data.xvelocity, data.yvelocity, data.zvelocity );

    // Save message in accumulator
    auto accumulatorKey = std::make_pair( static_cast<uint8_t>(MAVLINK_MSG_ID_ODOMETER_DELTA), static_cast<uint8_t>(odometer->sensorId()) );
    accumulator[accumulatorKey].push_back(odometer);
}

void MavlinkToData::parseOdometerDelta(const mavlink_message_t &msg){
    mavlink_odometer_delta_t data;
    mavlink_msg_odometer_delta_decode(&msg,&data);

    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();

    odometer->sensorId(msg.compid);
    const auto& cfg = getOdometerConfig(odometer->sensorId(), false);
    odometer->name(cfg.name);
    odometer->timestamp(timestamp);

    odometer->distance = sensor_utils::Odometer::Measurement( data.xdist, data.ydist, data.zdist );
    odometer->velocity = sensor_utils::Odometer::Measurement( data.xvelocity, data.yvelocity, data.zvelocity );

    // Save message in accumulator
    auto accumulatorKey = std::make_pair( static_cast<uint8_t>(MAVLINK_MSG_ID_ODOMETER_DELTA), static_cast<uint8_t>(odometer->sensorId()) );
    accumulator[accumulatorKey].push_back(odometer);
}

void MavlinkToData::parseProximity(const mavlink_message_t &msg){
    //TODO hier mitteln
    //get the data
    int sensor_id = msg.compid;
    mavlink_proximity_t data;
    mavlink_msg_proximity_decode(&msg,&data);

    std::shared_ptr<sensor_utils::DistanceSensor> sensor =std::make_shared<sensor_utils::DistanceSensor>();
    sensor->sensorId(sensor_id);
    const auto& cfg = getProximityConfig(sensor->sensorId(), false);
    sensor->name(cfg.name);
    sensor->timestamp(timestamp);

    sensor->distance = data.distance;
    sensor->direction =  cfg.direction;
    sensor->localPosition.x = cfg.x;
    sensor->localPosition.y = cfg.y;
    sensors->put(sensor);
}

void MavlinkToData::parseParking(const mavlink_message_t &msg){

    int sensor_id = msg.compid;
    mavlink_parking_lot_t data;
    mavlink_msg_parking_lot_decode(&msg, &data);

    std::shared_ptr<sensor_utils::ParkingSensor> sensor = std::make_shared<sensor_utils::ParkingSensor>();
    sensor->timestamp(timestamp);
    sensor->sensorId(sensor_id);
    const auto& cfg = getParkingLotConfig(sensor->sensorId(), false);
    sensor->name(cfg.name);

    sensor->position = data.parking_lot_position;
    sensor->size = data.parking_lot_size;
    sensors->put(sensor);
}


void MavlinkToData::parseHeartBeat(const mavlink_message_t &msg){
    mavlink_heartbeat_t data;
    mavlink_msg_heartbeat_decode(&msg,&data);
    //mavlink_get_channel_status
    // Sync timestamps
    {
        auto service = getService<timestamp_interpolator_service::TimestampInterpolatorService>("TIMESTAMP_INTERPOLATOR");
        if(service.isValid()){
            debugRcCarState->steering_front = data.rc_steering_front;
            debugRcCarState->steering_rear = data.rc_steering_rear;
            debugRcCarState->targetSpeed = data.rc_velocity;

            auto mavlinkTimestamp = service->canonical("MAVLINK", lms::Time::fromMicros(data.timestamp));
            service->sync("SYSTEM", "MAVLINK", lms::Time::now(), mavlinkTimestamp, false);
            service->sync("MAVLINK", "SENSOR", mavlinkTimestamp, timestamp, false);
        }
    }

    {
        auto service = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
        if(service.isValid())
        {
            //get rc state
            phoenix_CC2016_service::RemoteControlState rcState = phoenix_CC2016_service::RemoteControlState::UNKOWN;

            if( data.remote_control == REMOTE_CONTROL_STATUS_DISCONNECTED ){
                rcState =phoenix_CC2016_service::RemoteControlState::DISCONNECTED;
            } else if( data.remote_control == REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS){
                rcState = phoenix_CC2016_service::RemoteControlState::REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS;
            }else if(data.remote_control == REMOTE_CONTROL_STATUS_AUTONOMOUS){
                rcState =phoenix_CC2016_service::RemoteControlState::REMOTE_CONTROL_STATUS_AUTONOMOUS;
            }else if(data.remote_control == REMOTE_CONTROL_STATUS_MANUAL){
                rcState =phoenix_CC2016_service::RemoteControlState::REMOTE_CONTROL_STATUS_MANUAL;
            }else{
                logger.error("parseHeartBeat")<<"invalid rc-state: "<<data.remote_control;
            }

            phoenix_CC2016_service::CCDriveMode driveMode = phoenix_CC2016_service::CCDriveMode::IDLE;
            logger.debug("drivemode")<<(int)driveMode;
            if(data.drive_mode == DRIVE_MODE_TRACK){
                driveMode = phoenix_CC2016_service::CCDriveMode::FOH;
            }else if(data.drive_mode == DRIVE_MODE_TRACK_OBSTACLES){
                driveMode = phoenix_CC2016_service::CCDriveMode::FMH;
            }else if(data.drive_mode == DRIVE_MODE_PARKING){
                driveMode = phoenix_CC2016_service::CCDriveMode::PARKING;
            }

            //TODO get values
            service->update(rcState,driveMode,data.battery_voltage);
            logger.debug("car drivemode")<<(int)service->driveMode();
        }else{
            logger.error("Phoenix service not valid!");
        }
    }
}

void MavlinkToData::accumulateIMU(uint8_t sensorId, MavlinkToData::SensorAccumulator &samples){
    std::shared_ptr<sensor_utils::IMU> imu = std::make_shared<sensor_utils::IMU>();

    // Set stuff from config
    const auto& cfg = getIMUConfig(imu->sensorId(), false);

    imu->sensorId(sensorId);
    imu->name(cfg.name);
    imu->timestamp(timestamp);

    imu->accelerometer.setZero();
    imu->gyroscope.setZero();
    imu->magnetometer.setZero();

    for(const auto& s : samples)
    {
        auto i = static_cast<sensor_utils::IMU*>( s.get() );
        imu->accelerometer  += i->accelerometer;
        imu->gyroscope      += i->gyroscope;
        imu->magnetometer   += i->magnetometer;
    }

    if( samples.size() > 0 )
    {
        // Average across samples
        imu->accelerometer /= samples.size();
        imu->gyroscope /= samples.size();
        imu->magnetometer /= samples.size();

        imu->accelerometerCovariance = cfg.accelerometerCovariance;
        imu->gyroscopeCovariance = cfg.gyroscopeCovariance;
        imu->magnetometerCovariance = cfg.magnetometerCovariance;

        sensors->put(imu);
    }

    // Clear accumulated messages
    samples.clear();
}

void MavlinkToData::accumulateOdometer(uint8_t sensorId, MavlinkToData::SensorAccumulator &samples)
{
    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();

    // Set stuff from config
    const auto& cfg = getOdometerConfig(sensorId, false);

    odometer->sensorId(sensorId);
    odometer->name(cfg.name);
    odometer->timestamp(timestamp);

    odometer->distance.setZero();
    odometer->velocity.setZero();

    for(const auto& s : samples)
    {
        auto i = static_cast<sensor_utils::Odometer*>( s.get() );
        odometer->distance  += i->distance;
        odometer->velocity  += i->velocity;
    }

    odometer->distanceCovariance = cfg.distanceCovariance;
    odometer->velocityCovariance = cfg.velocityCovariance;

    if( samples.size() > 0 )
    {
        odometer->velocity /= samples.size();

        // TODO: covariances, quality, absolute distance

        sensors->put(odometer);
    }

    // Clear accumulated messages
    samples.clear();
}

void MavlinkToData::computeCurrentTimestamp()
{

    switch(timebase) {
        case Timebase::FIXED:
            // Fixed pre-defined time-base (in hertz)
            {
                const auto tickrate = config().get<float>("sensor_timebase_tickrate", 100);
                const auto usPerTick = static_cast<lms::Time::TimeType>( (1.f / tickrate) * 1e6 );
                rawTimestamp += lms::Time::fromMicros(usPerTick);
            }
            break;
        case Timebase::SYSTEM:
            // LMS system time
            rawTimestamp = lms::Time::now();
            break;
        case Timebase::MAVLINK:
            // Mavlink timebase
            // Iterate over messages and take first message with valid timestamp
            {
                for( const auto& msg : *inChannel )
                {
                    if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
                        rawTimestamp = lms::Time::fromMicros(mavlink_msg_heartbeat_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_IMU){
                        rawTimestamp = lms::Time::fromMicros(mavlink_msg_imu_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA){
                        rawTimestamp = lms::Time::fromMicros(mavlink_msg_odometer_delta_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_PROXIMITY){
                        rawTimestamp = lms::Time::fromMicros(mavlink_msg_proximity_get_timestamp(&msg));
                        break;
                    }else if(msg.msgid == MAVLINK_MSG_ID_PARKING_LOT){
                        rawTimestamp = lms::Time::fromMicros(mavlink_msg_parking_lot_get_timestamp(&msg));
                        break;
                    }
                }
            }
            break;
    }

    // Sync timebases
    {
        auto service = getService<timestamp_interpolator_service::TimestampInterpolatorService>("TIMESTAMP_INTERPOLATOR");
        if(service.isValid())
        {
            // Make sensor timestamp canonical (overflow compensated)
            timestamp = service->canonical("SENSOR", rawTimestamp);


            // Sync system timebase to sensor timebase
            service->sync("SYSTEM", "SENSOR", lms::Time::now(), timestamp, false);
        }
    }
}

void MavlinkToData::configsChanged()
{
    // Reload all configs
    for(auto& c : imuConfigs ) {
        getIMUConfig(c.first, true);
    }
    for(auto& c : odometerConfigs ) {
        getOdometerConfig(c.first, true);
    }
    for(auto& c : proximityConfigs ) {
        getProximityConfig(c.first, true);
    }
}

const IMUConfig& MavlinkToData::getIMUConfig(size_t id, bool forceReload)
{
    if(forceReload || imuConfigs.find(id) == imuConfigs.end()) {
        // Config not found or not up-to-date

        IMUConfig cfg;
        std::string sensor = "imu_" + std::to_string(id);

        // Name
        cfg.name = config().get<std::string>(sensor, "IMU_" + std::to_string(id));

        // Biases
        cfg.gyroBias = sensor_utils::IMU::Measurement(
                config().get<float>(sensor + "_gyro_bias_x", 0),
                config().get<float>(sensor + "_gyro_bias_y", 0),
                config().get<float>(sensor + "_gyro_bias_z", 0)
        );
        cfg.accBias = sensor_utils::IMU::Measurement(
                config().get<float>(sensor + "_acc_bias_x", 0),
                config().get<float>(sensor + "_acc_bias_y", 0),
                config().get<float>(sensor + "_acc_bias_z", 0)
        );

        // Covariances
        cfg.accelerometerCovariance = sensor_utils::IMU::Covariance(
                config().get<float>(sensor + "_acc_cov_xx", 1),
                config().get<float>(sensor + "_acc_cov_xy", 0),
                config().get<float>(sensor + "_acc_cov_xz", 0),
                config().get<float>(sensor + "_acc_cov_xy", 0),
                config().get<float>(sensor + "_acc_cov_yy", 1),
                config().get<float>(sensor + "_acc_cov_yz", 0),
                config().get<float>(sensor + "_acc_cov_xz", 0),
                config().get<float>(sensor + "_acc_cov_xz", 0),
                config().get<float>(sensor + "_acc_cov_zz", 1)
        );
        cfg.gyroscopeCovariance = sensor_utils::IMU::Covariance(
                config().get<float>(sensor + "_gyro_cov_xx", 1),
                config().get<float>(sensor + "_gyro_cov_xy", 0),
                config().get<float>(sensor + "_gyro_cov_xz", 0),
                config().get<float>(sensor + "_gyro_cov_xy", 0),
                config().get<float>(sensor + "_gyro_cov_yy", 1),
                config().get<float>(sensor + "_gyro_cov_yz", 0),
                config().get<float>(sensor + "_gyro_cov_xz", 0),
                config().get<float>(sensor + "_gyro_cov_yz", 0),
                config().get<float>(sensor + "_gyro_cov_zz", 1)
        );
        cfg.magnetometerCovariance = sensor_utils::IMU::Covariance(
                config().get<float>(sensor + "_mag_cov_xx", 1),
                config().get<float>(sensor + "_mag_cov_xy", 0),
                config().get<float>(sensor + "_mag_cov_xz", 0),
                config().get<float>(sensor + "_mag_cov_xy", 0),
                config().get<float>(sensor + "_mag_cov_yy", 1),
                config().get<float>(sensor + "_mag_cov_yz", 0),
                config().get<float>(sensor + "_mag_cov_xz", 0),
                config().get<float>(sensor + "_mag_cov_yz", 0),
                config().get<float>(sensor + "_mag_cov_zz", 1)
        );

        // Set config
        imuConfigs[id] = cfg;
    }
    return imuConfigs[id];
}

const OdometerConfig& MavlinkToData::getOdometerConfig(size_t id, bool forceReload)
{
    if(forceReload || odometerConfigs.find(id) == odometerConfigs.end()) {
        // Config not found or not up-to-date

        OdometerConfig cfg;
        std::string sensor = "odometer_" + std::to_string(id);

        // Name
        cfg.name = config().get<std::string>(sensor, "ODOMETER_" + std::to_string(id));

        // Covariances
        cfg.distanceCovariance = sensor_utils::Odometer::Covariance(
                config().get<float>(sensor + "_dist_cov_xx", 1),
                config().get<float>(sensor + "_dist_cov_xy", 0),
                config().get<float>(sensor + "_dist_cov_xz", 0),
                config().get<float>(sensor + "_dist_cov_xy", 0),
                config().get<float>(sensor + "_dist_cov_yy", 1),
                config().get<float>(sensor + "_dist_cov_yz", 0),
                config().get<float>(sensor + "_dist_cov_xz", 0),
                config().get<float>(sensor + "_dist_cov_yz", 0),
                config().get<float>(sensor + "_dist_cov_zz", 1)
        );
        cfg.velocityCovariance = sensor_utils::Odometer::Covariance(
                config().get<float>(sensor + "_velo_cov_xx", 1),
                config().get<float>(sensor + "_velo_cov_xy", 0),
                config().get<float>(sensor + "_velo_cov_xz", 0),
                config().get<float>(sensor + "_velo_cov_xy", 0),
                config().get<float>(sensor + "_velo_cov_yy", 1),
                config().get<float>(sensor + "_velo_cov_yz", 0),
                config().get<float>(sensor + "_velo_cov_xz", 0),
                config().get<float>(sensor + "_velo_cov_yz", 0),
                config().get<float>(sensor + "_velo_cov_zz", 1)
        );

        // Set config
        odometerConfigs[id] = cfg;
    }
    return odometerConfigs[id];
}

const ProximityConfig& MavlinkToData::getProximityConfig(size_t id, bool forceReload)
{
    if(forceReload || proximityConfigs.find(id) == proximityConfigs.end()) {
        // Config not found or not up-to-date

        ProximityConfig cfg;
        std::string sensor = "distance_" + std::to_string(id);

        // Name
        cfg.name = config().get<std::string>(sensor, "DISTANCE_" + std::to_string(id));

        cfg.direction = config().get<float>(cfg.name+"_dir",0)*M_PI/180;
        cfg.x = config().get<float>(cfg.name+"_x",0);
        cfg.y = config().get<float>(cfg.name+"_y",0);

        // Set config
        proximityConfigs[id] = cfg;
    }
    return proximityConfigs[id];
}

const ParkingLotConfig& MavlinkToData::getParkingLotConfig(size_t id, bool forceReload)
{
    if(forceReload || parkingLotConfigs.find(id) == parkingLotConfigs.end()) {
        // Config not found or not up-to-date

        ParkingLotConfig cfg;
        std::string sensor = "parkingLot_" + std::to_string(id);

        // Name
        cfg.name = config().get<std::string>(sensor, "PARKINGLOT_" + std::to_string(id));

        // Set config
        parkingLotConfigs[id] = cfg;
    }
    return parkingLotConfigs[id];
}
