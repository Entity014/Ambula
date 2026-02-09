#include "odometry.h"

Odometry::Odometry()
{
    // default frames (เหมือนของเดิม)
    odom_msg_.header.frame_id =
        micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id =
        micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}

void Odometry::setFrames(const char *odom_frame, const char *base_frame)
{
    odom_msg_.header.frame_id =
        micro_ros_string_utilities_set(odom_msg_.header.frame_id, odom_frame);
    odom_msg_.child_frame_id =
        micro_ros_string_utilities_set(odom_msg_.child_frame_id, base_frame);
}

void Odometry::setGeometry(float wheel_radius_m, float wheel_separation_m)
{
    r_ = wheel_radius_m;
    L_ = wheel_separation_m;
}

void Odometry::setOmegaLpfAlpha(float alpha)
{
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    omega_alpha_ = alpha;
}

void Odometry::setUnwrapPosition(bool enable) { unwrap_position_ = enable; }
void Odometry::setDtMax(float dt_max_s) { dt_max_s_ = dt_max_s; }

void Odometry::setPoseCov(float cov_xy, float cov_yaw)
{
    pose_cov_xy_ = cov_xy;
    pose_cov_yaw_ = cov_yaw;
}

void Odometry::setTwistCov(float cov_v, float cov_yawrate)
{
    twist_cov_v_ = cov_v;
    twist_cov_yawrate_ = cov_yawrate;
}

void Odometry::reset(float x, float y, float yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = wrapPi(yaw);
    has_last_ = false;
    has_omega_f_ = false;
}

float Odometry::wrapPi(float a)
{
    // map to (-pi, pi]
    return fmodf(a + (float)M_PI, 2.0f * (float)M_PI) - (float)M_PI;
}

float Odometry::unwrapDelta(float dtheta)
{
    // map to (-pi, pi] to avoid wrap spikes
    return fmodf(dtheta + (float)M_PI, 2.0f * (float)M_PI) - (float)M_PI;
}

void Odometry::eulerToQuat(float roll, float pitch, float yaw, float *q)
{
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);

    // (w,x,y,z) packed in q[0..3]
    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

float Odometry::lpf(float x, float x_f) const
{
    // first-order IIR: x_f <- a*x + (1-a)*x_f
    return omega_alpha_ * x + (1.0f - omega_alpha_) * x_f;
}

bool Odometry::updateFromWheelPos(float pos_l_rad, float pos_r_rad, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > dt_max_s_)
    {
        // dt ไม่ดี: ไม่ integrate
        has_last_ = false; // รีเซ็ตเพื่อกัน spike รอบถัดไป
        return false;
    }

    if (!has_last_)
    {
        last_pos_l_ = pos_l_rad;
        last_pos_r_ = pos_r_rad;
        has_last_ = true;
        return false; // ยังไม่มี history พอจะหา omega
    }

    float dpl = pos_l_rad - last_pos_l_;
    float dpr = pos_r_rad - last_pos_r_;

    if (unwrap_position_)
    {
        dpl = unwrapDelta(dpl);
        dpr = unwrapDelta(dpr);
    }

    const float omega_l_raw = dpl / dt_s;
    const float omega_r_raw = dpr / dt_s;

    last_pos_l_ = pos_l_rad;
    last_pos_r_ = pos_r_rad;

    // LPF omegas (เหมือนใน Python)
    if (!has_omega_f_)
    {
        omega_l_f_ = omega_l_raw;
        omega_r_f_ = omega_r_raw;
        has_omega_f_ = true;
    }
    else
    {
        omega_l_f_ = lpf(omega_l_raw, omega_l_f_);
        omega_r_f_ = lpf(omega_r_raw, omega_r_f_);
    }

    // diff drive kinematics
    const float v = r_ * 0.5f * (omega_l_f_ + omega_r_f_);
    const float yaw_rate = r_ * (omega_r_f_ - omega_l_f_) / L_;

    // integrate pose (unicycle)
    x_ += v * cosf(yaw_) * dt_s;
    y_ += v * sinf(yaw_) * dt_s;
    yaw_ = wrapPi(yaw_ + yaw_rate * dt_s);

    // fill odom msg (เหมือน style ของ Jimeno)
    float q[4];
    eulerToQuat(0.0f, 0.0f, yaw_, q);

    odom_msg_.pose.pose.position.x = (double)x_;
    odom_msg_.pose.pose.position.y = (double)y_;
    odom_msg_.pose.pose.position.z = 0.0;

    odom_msg_.pose.pose.orientation.w = (double)q[0];
    odom_msg_.pose.pose.orientation.x = (double)q[1];
    odom_msg_.pose.pose.orientation.y = (double)q[2];
    odom_msg_.pose.pose.orientation.z = (double)q[3];

    // pose covariance (6x6 row-major)
    for (int i = 0; i < 36; i++)
        odom_msg_.pose.covariance[i] = 0.0;
    odom_msg_.pose.covariance[0] = pose_cov_xy_;   // x
    odom_msg_.pose.covariance[7] = pose_cov_xy_;   // y
    odom_msg_.pose.covariance[35] = pose_cov_yaw_; // yaw

    odom_msg_.twist.twist.linear.x = (double)v;
    odom_msg_.twist.twist.linear.y = 0.0;
    odom_msg_.twist.twist.linear.z = 0.0;
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = (double)yaw_rate;

    v_ = v;
    yaw_rate_ = yaw_rate;

    for (int i = 0; i < 36; i++)
        odom_msg_.twist.covariance[i] = 0.0;
    odom_msg_.twist.covariance[0] = twist_cov_v_;
    odom_msg_.twist.covariance[35] = twist_cov_yawrate_;

    return true;
}

nav_msgs__msg__Odometry Odometry::getData() const
{
    return odom_msg_;
}
