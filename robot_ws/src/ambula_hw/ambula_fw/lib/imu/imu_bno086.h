#ifndef IMU_BNO086_H
#define IMU_BNO086_H

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>

class BNO086IMU
{
protected:
    sensor_msgs__msg__Imu imu_msg_;
    const float g_to_accel_ = 9.81;
    const float mgauss_to_utesla_ = 0.1;
    const float utesla_to_tesla_ = 0.000001;

    float accel_cov_ = 0.00001;
    float gyro_cov_ = 0.00001;

    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;
    geometry_msgs__msg__Quaternion ori_;

    BNO08x bno086;

public:
    BNO086IMU()
    {
        imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");

        accel_.x = accel_.y = accel_.z = 0.0;
        gyro_.x = gyro_.y = gyro_.z = 0.0;
        ori_.x = ori_.y = ori_.z = 0.0;
        ori_.w = 1.0;
    }

    void pollOnce()
    {
        if (!bno086.getSensorEvent())
            return;

        switch (bno086.getSensorEventID())
        {
        case SENSOR_REPORTID_ACCELEROMETER:
            accel_.x = bno086.getAccelX(); // m/s^2
            accel_.y = bno086.getAccelY();
            accel_.z = bno086.getAccelZ();
            break;

        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
            gyro_.x = bno086.getGyroX(); // rad/s
            gyro_.y = bno086.getGyroY();
            gyro_.z = bno086.getGyroZ();
            break;

        case SENSOR_REPORTID_ROTATION_VECTOR:
            ori_.x = bno086.getQuatI();
            ori_.y = bno086.getQuatJ();
            ori_.z = bno086.getQuatK();
            ori_.w = bno086.getQuatReal();
            break;

        default:
            break;
        }
    }

    bool startSensor()
    {
        Wire.begin();

        bool ret;
        ret = bno086.begin(0x4A, Wire, -1, -1);

        if (!ret)
            return false;

        if (!bno086.enableAccelerometer())
            return false;
        if (!bno086.enableGyro())
            return false;
        if (!bno086.enableRotationVector())
            return false;

        return true;
    }

    bool init()
    {
        bool sensor_ok = startSensor();

        return sensor_ok;
    }

    sensor_msgs__msg__Imu getData()
    {
        pollOnce();
        imu_msg_.orientation = ori_;

        imu_msg_.angular_velocity = gyro_;
        imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

        imu_msg_.linear_acceleration = accel_;
        imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

        return imu_msg_;
    }
};

#endif