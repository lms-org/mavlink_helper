#include "mavlink_to_data.h"
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"
#include "sensor_utils/odometer.h"

#include <phoenix_CC2016_service/phoenix_CC2016_service.h>
#include <timestamp_interpolator_service/timestamp_interpolator_service.h>

bool MavlinkToData::initialize() {
    inChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    sensors = writeChannel<sensor_utils::SensorContainer>("SENSORS");

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

    logger.info("cycle")<<"parse errors: "<<mavlink_get_channel_status(0)->parse_error<<" packet drop: "<<mavlink_get_channel_status(0)->packet_rx_drop_count;

    return true;
}

void MavlinkToData::parseIncomingMessages(){

    for( const auto& msg : *inChannel ){
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
            parseHeartBeat(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_IMU){
            parseIMU(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA){
            parseOdometer(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_PROXIMITY){
            parseProximity(msg);
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
            case MAVLINK_MSG_ID_ODOMETER_DELTA:
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
    std::string sensor = "imu_" + std::to_string(imu->sensorId());
    imu->name(config().get<std::string>(sensor, "IMU_" + std::to_string(imu->sensorId())));
    imu->timestamp(timestamp);

    // Biases
    // TODO: add acc + mag bias as well
    // TODO: update only on config change for performance?
    auto gyroBias = sensor_utils::IMU::Measurement(
        config().get<float>(sensor + "_gyro_bias_x", 0),
        config().get<float>(sensor + "_gyro_bias_y", 0),
        config().get<float>(sensor + "_gyro_bias_z", 0)
    );

    // Measurements
    imu->accelerometer = sensor_utils::IMU::Measurement(data.xacc, data.yacc, data.zacc);
    imu->gyroscope     = sensor_utils::IMU::Measurement(data.xgyro, data.ygyro, data.zgyro) - gyroBias;
    imu->magnetometer  = sensor_utils::IMU::Measurement(data.xmag, data.ymag, data.zmag);

    // Save message in accumulator
    auto accumulatorKey = std::make_pair( static_cast<uint8_t>(MAVLINK_MSG_ID_IMU), static_cast<uint8_t>(imu->sensorId()) );
    accumulator[accumulatorKey].push_back(imu);
}

void MavlinkToData::parseOdometer(const mavlink_message_t &msg){
    mavlink_odometer_delta_t data;
    mavlink_msg_odometer_delta_decode(&msg,&data);

    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();

    odometer->sensorId(msg.compid);
    std::string sensor = "odometer_" + std::to_string(odometer->sensorId());
    odometer->name(config().get<std::string>(sensor, "ODOMETER_" + std::to_string(odometer->sensorId())));
    odometer->timestamp(timestamp);

    odometer->distance = sensor_utils::Odometer::Measurement( data.xdist, data.ydist, data.zdist );
    odometer->velocity = sensor_utils::Odometer::Measurement( data.xvelocity, data.yvelocity, data.zvelocity );

    // Save message in accumulator
    auto accumulatorKey = std::make_pair( static_cast<uint8_t>(MAVLINK_MSG_ID_ODOMETER_DELTA), static_cast<uint8_t>(odometer->sensorId()) );
    accumulator[accumulatorKey].push_back(odometer);
}

void MavlinkToData::parseProximity(const mavlink_message_t &msg){
    //get the data
    int sensor_id = msg.compid;
    mavlink_proximity_t data;
    mavlink_msg_proximity_decode(&msg,&data);

    std::shared_ptr<sensor_utils::DistanceSensor> sensor =std::make_shared<sensor_utils::DistanceSensor>();
    sensor->sensorId(sensor_id);
    std::string sensor_string = "distance_"+std::to_string(sensor_id);
    sensor->name(config().get<std::string>(sensor_string+"_name","DISTANCE_" + std::to_string(sensor_id)));
    sensor->timestamp(timestamp);

    sensor->distance = data.distance;
    sensor->direction = config().get<float>(sensor_string+"_direction",0);
    sensor->localPosition.x = config().get<float>(sensor_string+"_x",0);
    sensor->localPosition.y = config().get<float>(sensor_string+"_y",0);
    sensors->put(sensor);
}


void MavlinkToData::parseHeartBeat(const mavlink_message_t &msg){
    mavlink_heartbeat_t data;
    mavlink_msg_heartbeat_decode(&msg,&data);
    //mavlink_get_channel_status
    // Sync timestamps
    {
        auto service = getService<timestamp_interpolator_service::TimestampInterpolatorService>("TIMESTAMP_INTERPOLATOR");
        if(service.isValid())
        {
            // TODO: handle overflows? (-> interpolator service?)
            auto mavlinkTimestamp = lms::Time::fromMicros(data.timestamp);
            service->sync("SYSTEM", "MAVLINK", lms::Time::now(), mavlinkTimestamp);
            service->sync("MAVLINK", "SENSOR", mavlinkTimestamp, timestamp);
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
            if(data.drive_mode == DRIVE_MODE_TRACK){
                driveMode = phoenix_CC2016_service::CCDriveMode::FOH;
            }else if(data.drive_mode == DRIVE_MODE_TRACK_OBSTACLES){
                driveMode = phoenix_CC2016_service::CCDriveMode::FMH;
            }else if(data.drive_mode == DRIVE_MODE_PARKING){
                driveMode = phoenix_CC2016_service::CCDriveMode::PARKING;
            }

            //TODO get values
            service->update(rcState,driveMode,data.battery_voltage);
        }else{
            logger.error("Phoenix service not valid!");
        }
    }
}

void MavlinkToData::accumulateIMU(uint8_t sensorId, MavlinkToData::SensorAccumulator &samples){
    std::shared_ptr<sensor_utils::IMU> imu = std::make_shared<sensor_utils::IMU>();

    imu->sensorId(sensorId);
    std::string sensor = "imu_" + std::to_string(imu->sensorId());
    imu->name(config().get<std::string>(sensor, "IMU_" + std::to_string(imu->sensorId())));
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

        // Set covariances from config
        // TODO: load covariances once up-front and update only on config change (for performance)
        // TODO: adjust variances depending on sample count
        imu->accelerometerCovariance = sensor_utils::IMU::Covariance(
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
        imu->gyroscopeCovariance = sensor_utils::IMU::Covariance(
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
        imu->magnetometerCovariance = sensor_utils::IMU::Covariance(
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

        sensors->put(imu);
    }

    // Clear accumulated messages
    samples.clear();
}

void MavlinkToData::accumulateOdometer(uint8_t sensorId, MavlinkToData::SensorAccumulator &samples)
{
    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();

    odometer->sensorId(sensorId);
    std::string sensor = "odometer_" + std::to_string(odometer->sensorId());
    odometer->name(config().get<std::string>(sensor, "ODOMETER_" + std::to_string(odometer->sensorId())));
    odometer->timestamp(timestamp);

    odometer->distance.setZero();
    odometer->velocity.setZero();

    for(const auto& s : samples)
    {
        auto i = static_cast<sensor_utils::Odometer*>( s.get() );
        odometer->distance  += i->distance;
        odometer->velocity  += i->velocity;
    }

    odometer->distanceCovariance = sensor_utils::Odometer::Covariance(
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

    odometer->velocityCovariance = sensor_utils::Odometer::Covariance(
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

    if( samples.size() > 0 )
    {
        odometer->velocity /= samples.size();

        // TODO: covariances, quality

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
                timestamp += lms::Time::fromMicros(usPerTick);
            }
            break;
        case Timebase::SYSTEM:
            // LMS system time
            timestamp = lms::Time::now();
            break;
        case Timebase::MAVLINK:
            // Mavlink timebase
            // Iterate over messages and take first message with valid timestamp
            {
                for( const auto& msg : *inChannel )
                {
                    if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
                        timestamp = lms::Time::fromMicros(mavlink_msg_heartbeat_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_IMU){
                        timestamp = lms::Time::fromMicros(mavlink_msg_imu_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER_DELTA){
                        timestamp = lms::Time::fromMicros(mavlink_msg_odometer_delta_get_timestamp(&msg));
                        break;
                    } else if(msg.msgid == MAVLINK_MSG_ID_PROXIMITY){
                        timestamp = lms::Time::fromMicros(mavlink_msg_proximity_get_timestamp(&msg));
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
            // Sync system timebase to sensor timebase
            service->sync("SYSTEM", "SENSOR", lms::Time::now(), timestamp);
        }
    }
}
