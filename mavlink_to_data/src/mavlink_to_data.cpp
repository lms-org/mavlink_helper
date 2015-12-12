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
    sensor_utils::IMU imu;
    imu.sensorId(msg.compid);
    imu.name(config().get<std::string>("imu_"+std::to_string(imu.sensorId()),"UNKOWN"));
    //acc
    imu.acc.x = data.xacc;
    imu.acc.y = data.yacc;
    imu.acc.z = data.zacc;
    //gyro
    imu.gyro.x = data.xgyro;
    imu.gyro.y = data.ygyro;
    imu.gyro.z = data.zgyro;
    //mag
    imu.magnetometer.x = data.xmag;
    imu.magnetometer.y = data.ymag;
    imu.magnetometer.z = data.zmag;
    sensors->put(imu);

}

void MavlinkToData::parseOdometer(const mavlink_message_t &msg){
    mavlink_odometer_t data;
    mavlink_msg_odometer_decode(&msg,&data);
    std::string odoString = "odo_"+std::to_string(msg.compid);
    sensor_utils::Odometer odometer;
    odometer.name(config().get<std::string>(odoString+"_name","UNKOWN"));
    odometer.sensorId(msg.compid);
    odometer.xdist = data.xdist;
    odometer.ydist = data.ydist;
    odometer.zdist = data.zdist;
    odometer.xvelocity = data.xvelocity;
    odometer.yvelocity = data.yvelocity;
    odometer.zvelocity = data.zvelocity;
    sensors->put(odometer);


}

void MavlinkToData::parseProximity(const mavlink_message_t &msg){
    //get the data
    int sensor_id = msg.compid;
    mavlink_proximity_t data;
    mavlink_msg_proximity_decode(&msg,&data);

    sensor_utils::DistanceSensor sensor;
    sensor.sensorId(sensor_id);
    std::string sensor_string = "distance_"+std::to_string(sensor_id);
    sensor.name(config().get<std::string>(sensor_string+"_name","UNKOWN"));
    sensor.distance = data.distance;
    sensor.direction = config().get<float>(sensor_string+"_direction",0);
    sensor.localPosition.x = config().get<float>(sensor_string+"_x",0);
    sensor.localPosition.y = config().get<float>(sensor_string+"_y",0);
    sensors->put(sensor);
}


void MavlinkToData::parseHeartBeat(const mavlink_message_t &msg){
            mavlink_heartbeat_t data;
            mavlink_msg_heartbeat_decode(&msg,&data);
            // HEARTBEAT found
            lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> service = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");

            //get rc state
            phoenix_CC2016_service::RemoteControlState rcState = phoenix_CC2016_service::RemoteControlState::IDLE;

            if( data.remote_control == REMOTE_CONTROL_STATUS_DISCONNECTED ){
                rcState =phoenix_CC2016_service::RemoteControlState::DISCONNECTED;
            } else if( data.remote_control == REMOTE_CONTROL_STATUS_CONNECTED ){
                rcState =phoenix_CC2016_service::RemoteControlState::IDLE;
            }else if(data.remote_control == REMOTE_CONTROL_STATUS_ACTIVE){
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
