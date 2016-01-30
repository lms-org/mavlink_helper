#ifndef MAVLINK_TO_DATA_SENSORS_H
#define MAVLINK_TO_DATA_SENSORS_H

#include <sensor_utils/distance_sensor.h>
#include <sensor_utils/imu.h>
#include <sensor_utils/odometer.h>


struct IMUConfig {
    std::string name;
    sensor_utils::IMU::Measurement gyroBias;
    sensor_utils::IMU::Measurement accBias;
    sensor_utils::IMU::Covariance accelerometerCovariance;
    sensor_utils::IMU::Covariance gyroscopeCovariance;
    sensor_utils::IMU::Covariance magnetometerCovariance;

};

struct OdometerConfig {
    std::string name;
    sensor_utils::Odometer::Covariance distanceCovariance;
    sensor_utils::Odometer::Covariance velocityCovariance;
};

struct ProximityConfig {
    std::string name;
    float direction;
    float x;
    float y;

};

#endif
