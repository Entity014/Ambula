#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <array>
#include <chrono>

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

struct ProfileOut
{
    std::vector<double> t;
    std::vector<double> u;   // [0..1]
    std::vector<double> ud;  // du/dt
    std::vector<double> udd; // d2u/dt2
    std::vector<double> uj;  // d3u/dt3
};

static ProfileOut profile_rectangle(double T, double dt)
{
    ProfileOut out;
    T = std::max(T, 1e-6);
    dt = std::max(dt, 1e-4);
    int n = std::max(2, (int)std::ceil(T / dt) + 1);

    out.t.resize(n);
    out.u.resize(n);
    out.ud.resize(n);
    out.udd.assign(n, 0.0);
    out.uj.assign(n, 0.0);

    for (int i = 0; i < n; i++)
    {
        out.t[i] = i * (T / (n - 1));
        out.u[i] = out.t[i] / T;
        out.ud[i] = 1.0 / T;
    }
    out.u.back() = 1.0;
    out.ud.back() = 0.0;
    return out;
}

static ProfileOut profile_trapezoid(double vmax, double amax, double dt)
{
    ProfileOut out;
    vmax = std::max(vmax, 1e-9);
    amax = std::max(amax, 1e-9);
    dt = std::max(dt, 1e-4);

    // distance in u-space is 1.0
    double D_need = (vmax * vmax) / amax;

    double Ta, Tv, Da;
    if (1.0 >= D_need)
    {
        Ta = vmax / amax;
        Da = 0.5 * amax * Ta * Ta;
        double Dc = 1.0 - 2.0 * Da;
        Tv = Dc / vmax;
    }
    else
    {
        vmax = std::sqrt(amax * 1.0); // triangular
        Ta = vmax / amax;
        Tv = 0.0;
        Da = 0.5 * amax * Ta * Ta;
    }

    double T = 2.0 * Ta + Tv;
    int n = std::max(2, (int)std::ceil(T / dt) + 1);

    out.t.resize(n);
    out.u.assign(n, 0.0);
    out.ud.assign(n, 0.0);
    out.udd.assign(n, 0.0);
    out.uj.assign(n, 0.0);

    for (int i = 0; i < n; i++)
        out.t[i] = i * (T / (n - 1));

    for (int i = 0; i < n; i++)
    {
        double ti = out.t[i];
        if (ti < Ta)
        {
            out.udd[i] = amax;
            out.ud[i] = amax * ti;
            out.u[i] = 0.5 * amax * ti * ti;
        }
        else if (ti < Ta + Tv)
        {
            out.udd[i] = 0.0;
            out.ud[i] = vmax;
            out.u[i] = Da + vmax * (ti - Ta);
        }
        else
        {
            double td = ti - (Ta + Tv);
            out.udd[i] = -amax;
            out.ud[i] = vmax - amax * td;
            out.u[i] = Da + (1.0 - 2.0 * Da) + vmax * td - 0.5 * amax * td * td;
        }
    }

    for (int i = 1; i < n; i++)
    {
        double dt_i = out.t[i] - out.t[i - 1];
        out.uj[i] = (out.udd[i] - out.udd[i - 1]) / std::max(dt_i, 1e-9);
    }

    out.u.back() = 1.0;
    out.ud.back() = 0.0;
    out.udd.back() = 0.0;
    out.uj.back() = 0.0;
    return out;
}

// Jerk-limited 7-phase S-curve (u-space move: 0->1)
static ProfileOut profile_scurve(double vmax, double amax, double jmax, double dt)
{
    ProfileOut out;
    vmax = std::max(vmax, 1e-9);
    amax = std::max(amax, 1e-9);
    jmax = std::max(jmax, 1e-9);
    dt = std::max(dt, 1e-4);

    double Tj, Tc;
    if (vmax >= (amax * amax) / jmax)
    {
        Tj = amax / jmax;
        Tc = (vmax / amax) - Tj;
    }
    else
    {
        Tj = std::sqrt(vmax / jmax);
        Tc = 0.0;
    }

    double a_reached = std::min(amax, jmax * Tj);

    // accel-half
    double u1 = (1.0 / 6.0) * jmax * std::pow(Tj, 3);
    double ud1 = 0.5 * jmax * std::pow(Tj, 2);

    double u2 = u1 + ud1 * Tc + 0.5 * a_reached * std::pow(Tc, 2);
    double ud2 = ud1 + a_reached * Tc;

    double u3 = u2 + ud2 * Tj + 0.5 * a_reached * std::pow(Tj, 2) - (1.0 / 6.0) * jmax * std::pow(Tj, 3);
    double ud_peak = ud2 + a_reached * Tj - 0.5 * jmax * std::pow(Tj, 2);

    double u_no_cruise = 2.0 * u3;
    double Tv;

    if (u_no_cruise <= 1.0 + 1e-12)
    {
        Tv = (1.0 - u_no_cruise) / std::max(ud_peak, 1e-9);
    }
    else
    {
        // short move: reduce Tc via binary search
        Tv = 0.0;
        double lo = 0.0, hi = Tc;
        for (int it = 0; it < 60; it++)
        {
            double mid = 0.5 * (lo + hi);
            double u2m = u1 + ud1 * mid + 0.5 * a_reached * mid * mid;
            double ud2m = ud1 + a_reached * mid;
            double u3m = u2m + ud2m * Tj + 0.5 * a_reached * Tj * Tj - (1.0 / 6.0) * jmax * Tj * Tj * Tj;
            double u_nc = 2.0 * u3m;
            if (u_nc > 1.0)
                hi = mid;
            else
                lo = mid;
        }
        Tc = lo;

        u2 = u1 + ud1 * Tc + 0.5 * a_reached * Tc * Tc;
        ud2 = ud1 + a_reached * Tc;
        u3 = u2 + ud2 * Tj + 0.5 * a_reached * Tj * Tj - (1.0 / 6.0) * jmax * Tj * Tj * Tj;
        ud_peak = ud2 + a_reached * Tj - 0.5 * jmax * Tj * Tj;
        Tv = 0.0;
    }

    Tv = std::max(0.0, Tv);

    // phase boundaries
    double t1 = Tj;
    double t2 = Tj + Tc;
    double t3 = 2.0 * Tj + Tc;
    double t4 = t3 + Tv;
    double t5 = t4 + Tj;
    double t6 = t5 + Tc;
    double t7 = t6 + Tj;
    double T = t7;

    // boundary states
    double u_t3 = u3;
    double ud_t3 = ud_peak;
    double u_t4 = u_t3 + ud_peak * Tv;
    double ud_t4 = ud_peak;

    double u_t5 = u_t4 + ud_peak * Tj - (1.0 / 6.0) * jmax * std::pow(Tj, 3);
    double ud_t5 = ud_peak - 0.5 * jmax * std::pow(Tj, 2);

    double u_t6 = u_t5 + ud_t5 * Tc - 0.5 * a_reached * std::pow(Tc, 2);
    double ud_t6 = ud_t5 - a_reached * Tc;

    auto seg = [&](double t) -> std::array<double, 4>
    {
        double u, ud, udd, uj;

        if (t < t1)
        { // 1) +jerk
            double tau = t;
            uj = jmax;
            udd = uj * tau;
            ud = 0.5 * uj * tau * tau;
            u = (1.0 / 6.0) * uj * tau * tau * tau;
            return {u, ud, udd, uj};
        }
        if (t < t2)
        { // 2) +a const
            double tau = t - t1;
            uj = 0.0;
            udd = a_reached;
            ud = ud1 + udd * tau;
            u = u1 + ud1 * tau + 0.5 * udd * tau * tau;
            return {u, ud, udd, uj};
        }
        if (t < t3)
        { // 3) -jerk
            double tau = t - t2;
            uj = -jmax;
            udd = a_reached + uj * tau;
            ud = ud2 + a_reached * tau + 0.5 * uj * tau * tau;
            u = u2 + ud2 * tau + 0.5 * a_reached * tau * tau + (1.0 / 6.0) * uj * tau * tau * tau;
            return {u, ud, udd, uj};
        }
        if (t < t4)
        { // 4) cruise
            double tau = t - t3;
            uj = 0.0;
            udd = 0.0;
            ud = ud_t3;
            u = u_t3 + ud * tau;
            return {u, ud, udd, uj};
        }
        if (t < t5)
        { // 5) -jerk
            double tau = t - t4;
            uj = -jmax;
            udd = uj * tau;
            ud = ud_t4 + 0.5 * uj * tau * tau;
            u = u_t4 + ud_t4 * tau + (1.0 / 6.0) * uj * tau * tau * tau;
            return {u, ud, udd, uj};
        }
        if (t < t6)
        { // 6) -a const
            double tau = t - t5;
            uj = 0.0;
            udd = -a_reached;
            ud = ud_t5 + udd * tau;
            u = u_t5 + ud_t5 * tau + 0.5 * udd * tau * tau;
            return {u, ud, udd, uj};
        }
        { // 7) +jerk
            double tau = t - t6;
            uj = jmax;
            udd = -a_reached + uj * tau;
            ud = ud_t6 + (-a_reached) * tau + 0.5 * uj * tau * tau;
            u = u_t6 + ud_t6 * tau + 0.5 * (-a_reached) * tau * tau + (1.0 / 6.0) * uj * tau * tau * tau;
            return {u, ud, udd, uj};
        }
    };

    int n = std::max(2, (int)std::ceil(T / dt) + 1);
    out.t.resize(n);
    out.u.resize(n);
    out.ud.resize(n);
    out.udd.resize(n);
    out.uj.resize(n);

    for (int i = 0; i < n; i++)
        out.t[i] = i * (T / (n - 1));

    for (int i = 0; i < n; i++)
    {
        auto s = seg(out.t[i]);
        out.u[i] = s[0];
        out.ud[i] = s[1];
        out.udd[i] = s[2];
        out.uj[i] = s[3];
    }

    // safety: clamp + monotonic
    out.u[0] = clamp(out.u[0], 0.0, 1.0);
    for (int i = 1; i < n; i++)
    {
        out.u[i] = clamp(out.u[i], 0.0, 1.0);
        out.u[i] = std::max(out.u[i], out.u[i - 1]);
    }

    out.u.back() = 1.0;
    out.ud.back() = 0.0;
    out.udd.back() = 0.0;
    out.uj.back() = 0.0;
    return out;
}

class JointSpaceSmootherTwist : public rclcpp::Node
{
public:
    JointSpaceSmootherTwist() : Node("joint_space_smoother_twist")
    {
        // topics
        declare_parameter<std::string>("state_topic", "/joint_states/hardware");
        declare_parameter<std::string>("target_topic", "/ambula/left_leg_joint_cmd_point");       // Point
        declare_parameter<std::string>("out_topic", "/ambula/left_leg_joint_cmd_twist_smoothed"); // Twist

        // joint names in JointState: [waist, hip, knee]
        declare_parameter<std::vector<std::string>>("joint_names", {"waist", "hip", "knee"});

        // profile
        declare_parameter<std::string>("profile", "s_curve");                   // rectangle|trapezoid|s_curve
        declare_parameter<std::vector<double>>("v_max", {2.0, 2.0, 2.0});       // rad/s
        declare_parameter<std::vector<double>>("a_max", {10.0, 10.0, 10.0});    // rad/s^2
        declare_parameter<std::vector<double>>("j_max", {100.0, 100.0, 100.0}); // rad/s^3 (jerk limit in u-space mapping)

        // timing
        declare_parameter<double>("publish_rate", 200.0);
        declare_parameter<double>("dt_profile", 0.002);

        // safety / usability
        declare_parameter<double>("min_T", 0.15);                          // sec (กันสั้นเกิน)
        declare_parameter<bool>("use_target_velocity_if_provided", false); // ถ้า true จะพยายาม match qd* ด้วย (เวอร์ชันนี้ยังไม่ blend)

        state_topic_ = get_parameter("state_topic").as_string();
        target_topic_ = get_parameter("target_topic").as_string();
        out_topic_ = get_parameter("out_topic").as_string();

        joint_names_ = get_parameter("joint_names").as_string_array();
        if (joint_names_.size() != 3)
            throw std::runtime_error("joint_names ต้องมี 3 ตัว: [waist, hip, knee]");

        profile_ = get_parameter("profile").as_string();
        std::transform(profile_.begin(), profile_.end(), profile_.begin(), ::tolower);

        v_max_ = get_parameter("v_max").as_double_array();
        a_max_ = get_parameter("a_max").as_double_array();
        j_max_ = get_parameter("j_max").as_double_array();
        publish_rate_ = get_parameter("publish_rate").as_double();
        dt_profile_ = get_parameter("dt_profile").as_double();
        min_T_ = get_parameter("min_T").as_double();
        use_target_velocity_if_provided_ = get_parameter("use_target_velocity_if_provided").as_bool();

        auto norm = [&](std::vector<double> &x)
        {
            if (x.empty())
                x.push_back(1.0);
            if (x.size() != joint_names_.size())
                x.assign(joint_names_.size(), x[0]);
        };
        norm(v_max_);
        norm(a_max_);
        norm(j_max_);

        sub_state_ = create_subscription<sensor_msgs::msg::JointState>(
            state_topic_, rclcpp::QoS(10),
            std::bind(&JointSpaceSmootherTwist::cb_state, this, std::placeholders::_1));

        sub_target_ = create_subscription<geometry_msgs::msg::Point>(
            target_topic_, rclcpp::QoS(10),
            std::bind(&JointSpaceSmootherTwist::cb_target, this, std::placeholders::_1));

        pub_out_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, rclcpp::QoS(10));

        double period = 1.0 / std::max(publish_rate_, 1.0);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&JointSpaceSmootherTwist::on_timer, this));

        RCLCPP_INFO(get_logger(), "state_topic=%s", state_topic_.c_str());
        RCLCPP_INFO(get_logger(), "target_topic=%s (Point: x=waist,y=hip,z=knee)", target_topic_.c_str());
        RCLCPP_INFO(get_logger(), "out_topic=%s (Twist: lin=pos, ang=vel)", out_topic_.c_str());
        RCLCPP_INFO(get_logger(), "profile=%s publish_rate=%.1f dt_profile=%.4f min_T=%.3f",
                    profile_.c_str(), publish_rate_, dt_profile_, min_T_);
    }

private:
    void cb_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // build cache once (when we see first state)
        if (!idx_cached_)
        {
            idx_cache_.fill(std::numeric_limits<size_t>::max());
            std::unordered_map<std::string, size_t> idx;
            idx.reserve(msg->name.size());
            for (size_t i = 0; i < msg->name.size(); i++)
                idx[msg->name[i]] = i;

            bool ok = true;
            for (size_t k = 0; k < 3; k++)
            {
                auto it = idx.find(joint_names_[k]);
                if (it == idx.end() || it->second >= msg->position.size())
                {
                    ok = false;
                    continue;
                }
                idx_cache_[k] = it->second;
            }
            if (ok)
            {
                idx_cached_ = true;
                RCLCPP_INFO(get_logger(), "JointState index cache ready.");
            }
        }

        if (!idx_cached_)
            return;
        if (msg->position.size() <= idx_cache_[0] ||
            msg->position.size() <= idx_cache_[1] ||
            msg->position.size() <= idx_cache_[2])
            return;

        // store current q
        q_current_[0] = msg->position[idx_cache_[0]];
        q_current_[1] = msg->position[idx_cache_[1]];
        q_current_[2] = msg->position[idx_cache_[2]];
        have_state_ = true;
    }

    void cb_target(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (!have_state_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No joint state yet; wait for state_topic.");
            return;
        }

        // target q*
        double q_star[3] = {msg->x, msg->y, msg->z};

        // q0 from measured
        double q0[3] = {q_current_[0], q_current_[1], q_current_[2]};
        double dq[3] = {q_star[0] - q0[0], q_star[1] - q0[1], q_star[2] - q0[2]};

        bool all_small = (std::fabs(dq[0]) < 1e-9 && std::fabs(dq[1]) < 1e-9 && std::fabs(dq[2]) < 1e-9);
        if (all_small)
            return;

        // conservative scalar limits in u-space
        double u_vmax = std::numeric_limits<double>::infinity();
        double u_amax = std::numeric_limits<double>::infinity();
        double u_jmax = std::numeric_limits<double>::infinity();

        for (int k = 0; k < 3; k++)
        {
            double d = std::fabs(dq[k]);
            if (d < 1e-9)
                continue;
            u_vmax = std::min(u_vmax, v_max_[k] / d);
            u_amax = std::min(u_amax, a_max_[k] / d);
            u_jmax = std::min(u_jmax, j_max_[k] / d);
        }

        if (!std::isfinite(u_vmax) || !std::isfinite(u_amax))
            return;
        if (!std::isfinite(u_jmax))
            u_jmax = 1e9;

        ProfileOut prof;
        if (profile_ == "rectangle")
        {
            double T = 1.0 / std::max(u_vmax, 1e-9);
            T = std::max(T, min_T_);
            prof = profile_rectangle(T, dt_profile_);
        }
        else if (profile_ == "trapezoid")
        {
            prof = profile_trapezoid(u_vmax, u_amax, dt_profile_);
            if (!prof.t.empty() && prof.t.back() < min_T_)
                prof = profile_trapezoid(std::max(1.0 / min_T_, 1e-9), u_amax, dt_profile_);
        }
        else
        {
            prof = profile_scurve(u_vmax, u_amax, u_jmax, dt_profile_);
            if (!prof.t.empty() && prof.t.back() < min_T_)
                prof = profile_scurve(std::max(1.0 / min_T_, 1e-9), u_amax, u_jmax, dt_profile_);
        }

        // store trajectory
        for (int k = 0; k < 3; k++)
        {
            q0_[k] = q0[k];
            dq_[k] = dq[k];
        }
        t_arr_ = std::move(prof.t);
        u_arr_ = std::move(prof.u);
        ud_arr_ = std::move(prof.ud);

        // last safety clamps
        if (!u_arr_.empty())
        {
            u_arr_[0] = clamp(u_arr_[0], 0.0, 1.0);
            for (size_t i = 1; i < u_arr_.size(); i++)
            {
                u_arr_[i] = clamp(u_arr_[i], 0.0, 1.0);
                u_arr_[i] = std::max(u_arr_[i], u_arr_[i - 1]);
            }
            u_arr_.back() = 1.0;
            if (!ud_arr_.empty())
                ud_arr_.back() = 0.0;
        }

        idx_ = 0;
        active_ = true;
        start_tp_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(),
                    "New target -> %s. T=%.4fs u_vmax=%.4g u_amax=%.4g u_jmax=%.4g",
                    profile_.c_str(),
                    t_arr_.empty() ? 0.0 : t_arr_.back(),
                    u_vmax, u_amax, u_jmax);
    }

    // interpolate u,ud at time elapsed
    bool sample_u(double elapsed, double &u, double &ud) const
    {
        if (t_arr_.empty() || u_arr_.empty())
            return false;

        // advance idx_ outside? เราจะให้ on_timer จัด idx_ แล้ว interpolate
        // แต่เพื่อ safety ทำ binary-like forward scan
        int i = idx_;
        while (i + 1 < (int)t_arr_.size() && t_arr_[i + 1] <= elapsed)
            i++;

        // clamp end
        if (i >= (int)t_arr_.size() - 1)
        {
            u = 1.0;
            ud = 0.0;
            return true;
        }

        double t0 = t_arr_[i];
        double t1 = t_arr_[i + 1];
        double alpha = (elapsed - t0) / std::max(t1 - t0, 1e-9);
        alpha = clamp(alpha, 0.0, 1.0);

        double u0 = u_arr_[i];
        double u1v = u_arr_[i + 1];
        u = u0 + (u1v - u0) * alpha;

        if (ud_arr_.size() == u_arr_.size())
        {
            double ud0 = ud_arr_[i];
            double ud1 = ud_arr_[i + 1];
            ud = ud0 + (ud1 - ud0) * alpha;
        }
        else
        {
            // fallback: finite diff u
            ud = (u1v - u0) / std::max(t1 - t0, 1e-9);
        }

        return true;
    }

    void on_timer()
    {
        if (!active_)
            return;

        auto now_tp = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now_tp - start_tp_).count();

        // keep idx_ roughly synced (fast forward)
        while (idx_ + 1 < (int)t_arr_.size() && t_arr_[idx_ + 1] <= elapsed)
            idx_++;

        double u = 0.0, ud = 0.0;
        if (!sample_u(elapsed, u, ud))
            return;

        // output Twist: lin=pos, ang=vel
        geometry_msgs::msg::Twist out;

        // q = q0 + u*dq
        // qd = ud*dq
        out.linear.x = q0_[0] + u * dq_[0];
        out.linear.y = q0_[1] + u * dq_[1];
        out.linear.z = q0_[2] + u * dq_[2];

        out.angular.x = ud * dq_[0];
        out.angular.y = ud * dq_[1];
        out.angular.z = ud * dq_[2];

        pub_out_->publish(out);

        // optional: update internal "current" prediction (helps chaining)
        q_current_[0] = out.linear.x;
        q_current_[1] = out.linear.y;
        q_current_[2] = out.linear.z;

        // done?
        if (idx_ >= (int)t_arr_.size() - 1 || elapsed >= t_arr_.back())
            active_ = false;
    }

private:
    // params
    std::string state_topic_, target_topic_, out_topic_, profile_;
    std::vector<std::string> joint_names_;
    std::vector<double> v_max_, a_max_, j_max_;
    double publish_rate_{200.0}, dt_profile_{0.002};
    double min_T_{0.15};
    bool use_target_velocity_if_provided_{false};

    // state
    bool have_state_{false};
    bool idx_cached_{false};
    std::array<size_t, 3> idx_cache_;
    double q_current_[3]{0.0, 0.0, 0.0};

    // trajectory
    bool active_{false};
    double q0_[3]{0.0, 0.0, 0.0};
    double dq_[3]{0.0, 0.0, 0.0};
    std::vector<double> t_arr_, u_arr_, ud_arr_;
    int idx_{0};
    std::chrono::steady_clock::time_point start_tp_;

    // ros
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_out_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSpaceSmootherTwist>());
    rclcpp::shutdown();
    return 0;
}
