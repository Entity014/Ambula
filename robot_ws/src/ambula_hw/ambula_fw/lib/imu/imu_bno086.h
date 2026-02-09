#ifndef IMU_BNO086_H
#define IMU_BNO086_H

#include <Wire.h>
#include <math.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#include <rosidl_runtime_c/string_functions.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>

class BNO086IMU
{
protected:
    sensor_msgs__msg__Imu imu_msg_;

    float accel_cov_ = 0.00001f;
    float gyro_cov_ = 0.00001f;

    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;
    geometry_msgs__msg__Quaternion ori_;

    // cached gravity (optional)
    float gxb_ = 0.0f, gyb_ = 0.0f, gzb_ = 1.0f;

    // timing + freshness
    uint32_t last_update_us_ = 0;
    bool has_new_ = false;

    BNO08x bno086;

    // -------------------- math helpers --------------------
    static inline void normalizeQuat(geometry_msgs__msg__Quaternion &q)
    {
        float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (n < 1e-12f)
        {
            q.x = q.y = q.z = 0.0f;
            q.w = 1.0f;
            return;
        }
        q.x /= n;
        q.y /= n;
        q.z /= n;
        q.w /= n;
    }

    static inline float wrapPi(float a)
    {
        while (a > (float)M_PI)
            a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI)
            a += 2.0f * (float)M_PI;
        return a;
    }

    static inline void gravityBodyFromQuat(
        const geometry_msgs__msg__Quaternion &q_in,
        float &gx, float &gy, float &gz)
    {
        geometry_msgs__msg__Quaternion q = q_in;
        normalizeQuat(q);

        gx = 2.0f * (q.x * q.z - q.w * q.y);
        gy = 2.0f * (q.w * q.x + q.y * q.z);
        gz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

        // normalize
        float n = sqrtf(gx * gx + gy * gy + gz * gz);
        if (n > 1e-9f)
        {
            gx /= n;
            gy /= n;
            gz /= n;
        }

        gx = -gx;
        gy = -gy;
        gz = -gz;
    }

    static inline void rollPitchFromGravity(float gx, float gy, float gz, float &roll, float &pitch)
    {
        roll = atan2f(gy, gz);
        pitch = atan2f(-gx, sqrtf(gy * gy + gz * gz));
        roll = wrapPi(roll);
        pitch = wrapPi(pitch);
    }

    static inline float yawFromQuat(const geometry_msgs__msg__Quaternion &q_in)
    {
        geometry_msgs__msg__Quaternion q = q_in;
        normalizeQuat(q);

        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        return wrapPi(atan2f(siny_cosp, cosy_cosp));
    }

    // update cached gravity after ori_ changed
    inline void updateGravityCache()
    {
        gravityBodyFromQuat(ori_, gxb_, gyb_, gzb_);
    }

public:
    BNO086IMU()
    {
        rosidl_runtime_c__String__init(&imu_msg_.header.frame_id);
        rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, "imu_link");

        accel_.x = accel_.y = accel_.z = 0.0f;
        gyro_.x = gyro_.y = gyro_.z = 0.0f;
        ori_.x = ori_.y = ori_.z = 0.0f;
        ori_.w = 1.0f;

        imu_msg_.orientation_covariance[0] = -1.0; // unknown orientation covariance in ROS IMU msg
        last_update_us_ = micros();
    }

    bool startSensor()
    {
        Wire.begin();

        if (!bno086.begin(0x4A, Wire, -1, -1))
            return false;
        if (!bno086.enableAccelerometer())
            return false;
        if (!bno086.enableGyroIntegratedRotationVector())
            return false;

        return true;
    }

    bool init() { return startSensor(); }

    // ===================== SINGLE POLL POINT =====================
    // Call at fixed rate in your main loop (e.g., 200-500Hz).
    // Drain all pending events so cache is freshest.
    void update()
    {
        has_new_ = false;

        // Drain event queue (อ่านจนหมด)
        noInterrupts();
        while (bno086.getSensorEvent())
        {
            has_new_ = true;

            switch (bno086.getSensorEventID())
            {
            case SENSOR_REPORTID_ACCELEROMETER:
                accel_.x = bno086.getAccelX();
                accel_.y = bno086.getAccelY();
                accel_.z = bno086.getAccelZ();
                break;

            case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR:
                gyro_.x = bno086.getGyroIntegratedRVangVelX();
                gyro_.y = bno086.getGyroIntegratedRVangVelY();
                gyro_.z = bno086.getGyroIntegratedRVangVelZ();

                ori_.x = bno086.getGyroIntegratedRVI();
                ori_.y = bno086.getGyroIntegratedRVJ();
                ori_.z = bno086.getGyroIntegratedRVK();
                ori_.w = bno086.getGyroIntegratedRVReal();

                updateGravityCache();
                break;

            default:
                break;
            }
        }

        if (has_new_)
            last_update_us_ = micros();
        interrupts();
    }

    bool hasNew() const { return has_new_; }
    uint32_t lastUpdateUs() const { return last_update_us_; }

    // ===================== GETTERS (NO POLL INSIDE) =====================
    sensor_msgs__msg__Imu getData()
    {
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

    void getGravityBody(float &gx, float &gy, float &gz) const
    {
        gx = gxb_;
        gy = gyb_;
        gz = gzb_;
    }

    float getRoll() const
    {
        float roll, pitch;
        rollPitchFromGravity(gxb_, gyb_, gzb_, roll, pitch);
        return roll;
    }

    float getPitch() const
    {
        float roll, pitch;
        rollPitchFromGravity(gxb_, gyb_, gzb_, roll, pitch);
        return pitch;
    }

    float getYaw() const
    {
        return yawFromQuat(ori_);
    }

    float getRollDeg() const { return getRoll() * 180.0f / (float)M_PI; }
    float getPitchDeg() const { return getPitch() * 180.0f / (float)M_PI; }
    float getYawDeg() const { return getYaw() * 180.0f / (float)M_PI; }

    // Angular rates (rad/s)
    float getRollRate() const { return gyro_.x; }
    float getPitchRate() const { return gyro_.y; } // << ตรวจแกนให้ตรงการติดตั้งจริง
    float getYawRate() const { return gyro_.z; }

    // Deg/s helpers (optional)
    float getRollRateDeg() const { return gyro_.x * 180.0f / (float)M_PI; }
    float getPitchRateDeg() const { return gyro_.y * 180.0f / (float)M_PI; }
    float getYawRateDeg() const { return gyro_.z * 180.0f / (float)M_PI; }
};

#endif
