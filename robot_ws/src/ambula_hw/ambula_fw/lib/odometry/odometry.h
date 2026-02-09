#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <math.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>

class Odometry
{
public:
    Odometry();

    // --- Config ---
    void setFrames(const char *odom_frame, const char *base_frame);
    void setGeometry(float wheel_radius_m, float wheel_separation_m);

    // Filtering & robustness
    void setOmegaLpfAlpha(float alpha); // 0..1 (higher = faster, lower = smoother)
    void setUnwrapPosition(bool enable);
    void setDtMax(float dt_max_s);

    // Covariances
    void setPoseCov(float cov_xy, float cov_yaw);
    void setTwistCov(float cov_v, float cov_yawrate);

    // Reset pose
    void reset(float x = 0.0f, float y = 0.0f, float yaw = 0.0f);

    /**
     * Update using wheel positions (rad) and a dt (sec).
     * - pos_l_rad / pos_r_rad : wheel joint position (rad) (absolute wrapping or continuous)
     * - dt_s : time step in seconds (use msg stamp dt if you have it; else fixed loop dt)
     *
     * Returns true if updated (enough history & dt valid), else false.
     */
    bool updateFromWheelPos(float pos_l_rad, float pos_r_rad, float dt_s);

    nav_msgs__msg__Odometry getData() const;

    float getVx() const { return v_; }
    float getYawRate() const { return yaw_rate_; }
    float getX() const { return x_; }
    float getY() const { return y_; }
    float getYaw() const { return yaw_; }

private:
    // helpers
    static float wrapPi(float a);
    static float unwrapDelta(float dtheta);
    static void eulerToQuat(float roll, float pitch, float yaw, float *q);
    float lpf(float x, float x_f) const;

private:
    nav_msgs__msg__Odometry odom_msg_;

    // geometry
    float r_ = 0.075f; // wheel radius [m]
    float L_ = 0.40f;  // wheel separation [m]

    // pose
    float x_ = 0.0f;
    float y_ = 0.0f;
    float yaw_ = 0.0f;

    // last wheel pos
    bool has_last_ = false;
    float last_pos_l_ = 0.0f;
    float last_pos_r_ = 0.0f;

    // filtered omegas
    bool has_omega_f_ = false;
    float omega_l_f_ = 0.0f;
    float omega_r_f_ = 0.0f;

    // options
    float omega_alpha_ = 0.35f;
    bool unwrap_position_ = true;
    float dt_max_s_ = 1.0f;

    // cov
    float pose_cov_xy_ = 0.02f;
    float pose_cov_yaw_ = 0.05f;
    float twist_cov_v_ = 0.05f;
    float twist_cov_yawrate_ = 0.05f;

    float v_ = 0.0f;
    float yaw_rate_ = 0.0f;
};

#endif
