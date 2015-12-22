#include "mavlink_to_data.h"
#include "sensor_utils/distance_sensor.h"
#include "sensor_utils/imu.h"
#include "sensor_utils/odometer.h"


bool MavlinkToData::initialize() {
    inChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    sensors = writeChannel<sensor_utils::SensorContainer>("SENSORS");
    return true;
}

bool MavlinkToData::deinitialize() {
    return true;
}

bool MavlinkToData::cycle() {
    parseIncomingMessages();
    return true;
}

void MavlinkToData::parseIncomingMessages(){

    for( const auto& msg : *inChannel ){
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
            parseHeartBeat(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_IMU){
            parseIMU(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_ODOMETER){
            parseOdometer(msg);
        }else if(msg.msgid == MAVLINK_MSG_ID_PROXIMITY){
            parseProximity(msg);
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

    // Measurements
    imu->accelerometer = sensor_utils::IMU::Measurement(data.xacc, data.yacc, data.zacc);
    imu->gyroscope     = sensor_utils::IMU::Measurement(data.xgyro, data.ygyro, data.zgyro);
    imu->magnetometer  = sensor_utils::IMU::Measurement(data.xmag, data.ymag, data.zmag);

    // Set covariances from config
    // TODO: load covariances once up-front and update only on config change (for performance)
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

void MavlinkToData::parseOdometer(const mavlink_message_t &msg){
    mavlink_odometer_t data;
    mavlink_msg_odometer_decode(&msg,&data);
    std::shared_ptr<sensor_utils::Odometer> odometer = std::make_shared<sensor_utils::Odometer>();
    odometer->sensorId(msg.compid);

    odometer->sensorId(msg.compid);
    std::string sensor = "odometer_" + std::to_string(odometer->sensorId());
    odometer->name(config().get<std::string>(sensor, "ODOMETER_" + std::to_string(odometer->sensorId())));

    odometer->distance = sensor_utils::Odometer::Measurement( data.xdist, data.ydist, data.zdist );
    odometer->distance = sensor_utils::Odometer::Measurement( data.xvelocity, data.yvelocity, data.zvelocity );

    // TODO: covariances, quality

    sensors->put(odometer);
}

void MavlinkToData::parseProximity(const mavlink_message_t &msg){
    //get the data
    int sensor_id = msg.compid;
    mavlink_proximity_t data;
    mavlink_msg_proximity_decode(&msg,&data);

    std::shared_ptr<sensor_utils::DistanceSensor> sensor =std::make_shared<sensor_utils::DistanceSensor>();
    sensor->sensorId(sensor_id);
    std::string sensor_string = "distance_"+std::to_string(sensor_id);
    sensor->name(config().get<std::string>(sensor_string+"_name","DISTANCE_"+sensor_id));
    sensor->distance = data.distance;
    sensor->direction = config().get<float>(sensor_string+"_direction",0);
    sensor->localPosition.x = config().get<float>(sensor_string+"_x",0);
    sensor->localPosition.y = config().get<float>(sensor_string+"_y",0);
    sensors->put(sensor);
}


void MavlinkToData::parseHeartBeat(const mavlink_message_t &msg){
    mavlink_heartbeat_t data;
    mavlink_msg_heartbeat_decode(&msg,&data);

    lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> service = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
    if(!service.isValid())
    {
        return;
    }

    //get rc state
    phoenix_CC2016_service::RemoteControlState rcState = phoenix_CC2016_service::RemoteControlState::IDLE;

    if( data.remote_control == REMOTE_CONTROL_STATUS_DISCONNECTED ){
        rcState =phoenix_CC2016_service::RemoteControlState::DISCONNECTED;
    } else if( data.remote_control == REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS || data.remote_control == REMOTE_CONTROL_STATUS_AUTONOMOUS ){
        rcState =phoenix_CC2016_service::RemoteControlState::IDLE;
    }else if(data.remote_control == REMOTE_CONTROL_STATUS_MANUAL){
        rcState =phoenix_CC2016_service::RemoteControlState::ACTIVE;
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
}
