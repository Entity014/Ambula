#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <IntervalTimer.h>

#include "config.h"
#include "lqr.h"
#include "imu.h"
#include "odometry.h"
#include "kinematics.h"
#include "leg_kinematics.h"
#include "servo_driver.h"

// ---------- ROS 2 / micro-ROS ----------
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <micro_ros_utilities/type_utilities.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

// ---------- ODrive CAN ----------
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // teensy compile hack

const float RAD2DEG = 180.0f / M_PI;
const float DEG2RAD = M_PI / 180.0f;

enum JointIndex
{
    J_LEFT_WAIST = 0,
    J_LEFT_HIP,
    J_LEFT_KNEE,
    J_LEFT_WHEEL,
    J_RIGHT_WAIST,
    J_RIGHT_HIP,
    J_RIGHT_KNEE,
    J_RIGHT_WHEEL
};

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Define > -------------------------------------//

rcl_publisher_t imu_publisher;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t joint_state_motor_publisher;
rcl_publisher_t left_foot_publisher;
rcl_publisher_t right_foot_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t left_leg_subscriber;
rcl_subscription_t right_leg_subscriber;
rcl_subscription_t robot_state_subscriber;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState joint_state_motor_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Point robot_state_msg;
geometry_msgs__msg__Point left_foot_msg;
geometry_msgs__msg__Point right_foot_msg;
geometry_msgs__msg__Twist left_leg_msg;
geometry_msgs__msg__Twist right_leg_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist debug_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0, payload_pulse_start_ms = 0;

enum connection_states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} connection_state;

enum robot_states
{
    IDLE,
    SLIDE,
    OPERATING,
} robot_state;

robot_states prev_robot_state = IDLE;

enum payload_states
{
    HOLD,
    DEPLOY,
} payload_state;

payload_states prev_payload_state = HOLD;

bool payload_pulsing = false;

enum light_states
{
    TURN_ON,
    BLINK,
    TURN_OFF,
} light_state;

light_states prev_light_state = TURN_ON;

Kinematics kinematics(
    Kinematics::DIFFERENTIAL_DRIVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    RPM_RATIO,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

LegKinematics::link_param LEG_LEFT_PARAM = {
    .L1 = LEG_L1,
    .L2 = LEG_L2,
    .L3 = LEG_L3,
    .hip_x = LEG_HIP_X,
    .hip_y = LEG_HIP_Y_LEFT,
    .hip_z = LEG_HIP_Z,
    .align_y = LEG_ALIGN_Y,

    .waist_off = WAIST_OFFSET_LEFT,
    .hip_off = HIP_OFFSET_LEFT,
    .knee_off = KNEE_OFFSET_LEFT,
};

LegKinematics::link_param LEG_RIGHT_PARAM = {
    .L1 = LEG_L1,
    .L2 = LEG_L2,
    .L3 = LEG_L3,
    .hip_x = LEG_HIP_X,
    .hip_y = LEG_HIP_Y_RIGHT,
    .hip_z = LEG_HIP_Z,
    .align_y = LEG_ALIGN_Y,

    .waist_off = WAIST_OFFSET_RIGHT,
    .hip_off = HIP_OFFSET_RIGHT,
    .knee_off = KNEE_OFFSET_RIGHT,
};

LegKinematics leg_left_fk(LEG_LEFT_PARAM);
LegKinematics leg_right_fk(LEG_RIGHT_PARAM);

IMU imuSensor;
Servo_Driver servoActuator(SERVOMIN, SERVOMAX, SERVO_FREQ);

Odometry odom;
bool have_wheel = false;
float wheel_pos_l_rad = 0.0f;
float wheel_pos_r_rad = 0.0f;
uint32_t last_odom_ms = 0;
bool got_left_wheel = false;
bool got_right_wheel = false;
//------------------------------ < LQR > ------------------------------//

static IntervalTimer lqr_timer;

// shared outputs from ISR
static volatile float tau_L_cmd = 0.0f;
static volatile float tau_R_cmd = 0.0f;

// shared references from ROS callback
static volatile float v_ref = 0.0f;
static volatile float r_ref = 0.0f;

// enable flag
static volatile bool lqr_enable = false;

static volatile float v_est = 0.0f;
static volatile bool have_wheel_est = false;

struct LqrGains
{
    float k00, k01, k02, k03; // tau_fwd
    float k10, k11, k12, k13; // tau_yaw
};

// ----- Scheduled gains shared with ISR -----
static volatile LqrGains K_sched = {
    -4.310300f, 0.357308f, -0.889553f, -0.000000f,
    0.0f, 0.0f, 0.0f, 0.165685f};

static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

static inline void getScheduledGains(float h, LqrGains &out)
{
    // clamp h into table range
    if (h <= AMBULA_H[0])
    {
        out.k00 = AMBULA_K[0][0][0];
        out.k01 = AMBULA_K[0][0][1];
        out.k02 = AMBULA_K[0][0][2];
        out.k03 = AMBULA_K[0][0][3];
        out.k10 = AMBULA_K[0][1][0];
        out.k11 = AMBULA_K[0][1][1];
        out.k12 = AMBULA_K[0][1][2];
        out.k13 = AMBULA_K[0][1][3];
        return;
    }
    if (h >= AMBULA_H[AMBULA_N - 1])
    {
        int i = AMBULA_N - 1;
        out.k00 = AMBULA_K[i][0][0];
        out.k01 = AMBULA_K[i][0][1];
        out.k02 = AMBULA_K[i][0][2];
        out.k03 = AMBULA_K[i][0][3];
        out.k10 = AMBULA_K[i][1][0];
        out.k11 = AMBULA_K[i][1][1];
        out.k12 = AMBULA_K[i][1][2];
        out.k13 = AMBULA_K[i][1][3];
        return;
    }

    // find segment [i, i+1]
    int i = 0;
    for (; i < AMBULA_N - 1; ++i)
    {
        if (h < AMBULA_H[i + 1])
            break;
    }

    float h0 = AMBULA_H[i];
    float h1 = AMBULA_H[i + 1];
    float t = (h - h0) / (h1 - h0);

    // interpolate all gains
    out.k00 = lerp(AMBULA_K[i][0][0], AMBULA_K[i + 1][0][0], t);
    out.k01 = lerp(AMBULA_K[i][0][1], AMBULA_K[i + 1][0][1], t);
    out.k02 = lerp(AMBULA_K[i][0][2], AMBULA_K[i + 1][0][2], t);
    out.k03 = lerp(AMBULA_K[i][0][3], AMBULA_K[i + 1][0][3], t);

    out.k10 = lerp(AMBULA_K[i][1][0], AMBULA_K[i + 1][1][0], t);
    out.k11 = lerp(AMBULA_K[i][1][1], AMBULA_K[i + 1][1][1], t);
    out.k12 = lerp(AMBULA_K[i][1][2], AMBULA_K[i + 1][1][2], t);
    out.k13 = lerp(AMBULA_K[i][1][3], AMBULA_K[i + 1][1][3], t);
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

static inline float dir_sign(int8_t d)
{
    return (d >= 0) ? 1.0f : -1.0f;
}

void lqr_isr()
{
    if (!lqr_enable)
    {
        tau_L_cmd = 0;
        tau_R_cmd = 0;
        return;
    }

    // อ่าน state แบบ atomic (สั้นมาก)
    float theta, theta_dot, r;
    noInterrupts();
    theta = imuSensor.getPitch();
    theta_dot = imuSensor.getPitchRate();
    r = imuSensor.getYawRate();
    interrupts();

    float v, vref, rref;
    bool wheel_ok;
    LqrGains Klocal;
    noInterrupts();
    v = v_est;
    wheel_ok = have_wheel_est;
    vref = v_ref;
    rref = r_ref;

    // copy field-by-field from volatile
    Klocal.k00 = K_sched.k00;
    Klocal.k01 = K_sched.k01;
    Klocal.k02 = K_sched.k02;
    Klocal.k03 = K_sched.k03;
    Klocal.k10 = K_sched.k10;
    Klocal.k11 = K_sched.k11;
    Klocal.k12 = K_sched.k12;
    Klocal.k13 = K_sched.k13;
    interrupts();

    if (!wheel_ok || fabsf(theta) > 0.7f)
    {
        tau_L_cmd = 0;
        tau_R_cmd = 0;
        return;
    }

    float x0 = (theta);
    float x1 = theta_dot;
    float x2 = (v - vref);
    float x3 = (r - rref);

    float tau_fwd = -(Klocal.k00 * x0 + Klocal.k01 * x1 + Klocal.k02 * x2 + Klocal.k03 * x3);
    float tau_yaw = -(Klocal.k10 * x0 + Klocal.k11 * x1 + Klocal.k12 * x2 + Klocal.k13 * x3);

    float tauL = tau_fwd - tau_yaw;
    float tauR = tau_fwd + tau_yaw;

    tauL = clampf(tauL, -TAU_MAX, TAU_MAX);
    tauR = clampf(tauR, -TAU_MAX, TAU_MAX);

    tau_L_cmd = tauL;
    tau_R_cmd = tauR;
}

static float h_filt = 0.16f;        // init กลางๆ
static const float H_ALPHA = 0.10f; // ปรับตาม rate ที่คุณเรียก update

static inline void updateGainScheduling(float h_hat)
{
    // filter height
    h_filt = h_filt + H_ALPHA * (h_hat - h_filt);
    h_filt = clampf(h_filt, AMBULA_H[0], AMBULA_H[AMBULA_N - 1]);

    // compute scheduled K
    LqrGains Knew;
    getScheduledGains(h_filt, Knew);

    // atomic swap ให้ ISR
    noInterrupts();
    K_sched = Knew;
    interrupts();
}

struct ImpGains
{
    float kp, kd, tau_max;
};
static ImpGains hip_g{1.0f, 0.0f, 0.5f}; // เริ่มต่ำๆก่อน
static ImpGains knee_g{3.0f, 0.1f, 1.0f};

static inline float sat(float x, float m) { return (x > m) ? m : (x < -m) ? -m
                                                                          : x; }

static inline float impedance_tau(float q, float qd, float q_des, float qd_des,
                                  const ImpGains &g, float tau_ff = 0.0f)
{
    float e = q_des - q;
    float ed = qd_des - qd;
    float tau = g.kp * e + g.kd * ed + tau_ff;
    return sat(tau, g.tau_max);
}

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void changeMode();
void moveBase();
void moveActuator();
void publishData();
bool createEntities();
bool destroyEntities();
void all_odrives_idle();
void wheels_closed_loop_only_safe();
void all_odrives_closed();
void wheels_set_velocity_mode();
void wheels_set_torque_mode();
void legs_set_position_mode();
void legs_set_torque_mode();

struct timespec getTime();

//------------------------------ < Rapid Define > -------------------------------------//

float waist_arr[2] = {90.f, 90.f}; // left, right

// ======================= CAN / ODrive ====================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

ODriveCAN odrv_right_hip(wrap_can_intf(can_intf), ODRV_RIGHT_HIP_ID);
ODriveCAN odrv_left_hip(wrap_can_intf(can_intf), ODRV_LEFT_HIP_ID);
ODriveCAN odrv_right_knee(wrap_can_intf(can_intf), ODRV_RIGHT_KNEE_ID);
ODriveCAN odrv_left_knee(wrap_can_intf(can_intf), ODRV_LEFT_KNEE_ID);
ODriveCAN odrv_right_wheel(wrap_can_intf(can_intf), ODRV_RIGHT_WHEEL_ID);
ODriveCAN odrv_left_wheel(wrap_can_intf(can_intf), ODRV_LEFT_WHEEL_ID);

ODriveCAN *ODRIVES[6] = {
    &odrv_right_hip, &odrv_left_hip, &odrv_right_knee,
    &odrv_left_knee, &odrv_right_wheel, &odrv_left_wheel};

struct ODrvData
{
    Heartbeat_msg_t hb{};
    Get_Encoder_Estimates_msg_t fb{};
    bool got_hb = false, got_fb = false;
};
ODrvData D_right_hip, D_left_hip, D_right_knee, D_left_knee, D_right_wheel, D_left_wheel;

struct JointCfg
{
    ODriveCAN *drv;
    ODrvData *data;
    int pos_idx;

    // scale (หน่วย base: rad หรือ deg แล้วแต่ pipeline)
    float pos_scale;
    float vel_scale;

    float pos_offset_rad;

    // flip: apply กับ raw input (เหมือนใน python)
    bool flip;

    // direction: apply ตอนท้ายสุดที่ output (+1/-1)
    int8_t direction;

    bool pos0_valid;
    float pos0_raw; // initial Pos_Estimate (turns)
    bool output_deg;
};

JointCfg maps[] = {
    {&odrv_left_hip, &D_left_hip, J_LEFT_HIP, 2.0f * M_PI / 20.0f, 2.0f * M_PI / 20.0f, 65.08f * DEG2RAD, true, -1, false, 0.0f, false},
    {&odrv_left_knee, &D_left_knee, J_LEFT_KNEE, 2.0f * M_PI / 10.0f, 2.0f * M_PI / 10.0f, 145.0f * DEG2RAD, false, -1, false, 0.0f, false},
    {&odrv_left_wheel, &D_left_wheel, J_LEFT_WHEEL, 2.0f * M_PI * 0.67f, 2.0f * M_PI * 0.67f, 0.0f, true, +1, false, 0.0f, false},
    {&odrv_right_hip, &D_right_hip, J_RIGHT_HIP, 2.0f * M_PI / 20.0f, 2.0f * M_PI / 20.0f, 65.88f * DEG2RAD, false, -1, false, 0.0f, false},
    {&odrv_right_knee, &D_right_knee, J_RIGHT_KNEE, 2.0f * M_PI / 10.0f, 2.0f * M_PI / 10.0f, 145.0f * DEG2RAD, true, -1, false, 0.0f, false},
    {&odrv_right_wheel, &D_right_wheel, J_RIGHT_WHEEL, 2.0f * M_PI * 0.67f, 2.0f * M_PI * 0.67f, 0.0f, false, +1, false, 0.0f, false},
};

static constexpr int JOINT_MAP_N = sizeof(maps) / sizeof(maps[0]);

bool request_state_with_retry(
    ODriveCAN &drv, ODrvData &ud, ODriveAxisState desired_state,
    uint32_t settle_ms,
    uint32_t per_try_timeout_ms,
    int max_retry);

void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
    auto *d = static_cast<ODrvData *>(user_data);
    d->hb = msg;
    d->got_hb = true;
}
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
    auto *d = static_cast<ODrvData *>(user_data);
    d->fb = msg;
    d->got_fb = true;
}
void onCanMessage(const CanMsg &m)
{
    for (auto *o : ODRIVES)
        onReceive(m, *o);
}

bool setupCan()
{
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}

static bool capture_joint_zero(uint32_t timeout_ms = 800)
{
    uint32_t t0 = millis();
    bool all_ok = false;

    while (millis() - t0 < timeout_ms)
    {
        pumpEvents(can_intf);

        int ok_cnt = 0;
        for (int i = 0; i < JOINT_MAP_N; ++i)
        {
            auto &m = maps[i];
            if (m.pos0_valid)
            {
                ok_cnt++;
                continue;
            }

            Get_Encoder_Estimates_msg_t fb;
            if (m.drv->getFeedback(fb, 3))
            {
                m.pos0_raw = fb.Pos_Estimate; // turns
                m.pos0_valid = true;
                ok_cnt++;
            }
        }

        if (ok_cnt == JOINT_MAP_N)
        {
            all_ok = true;
            break;
        }
        delay(5);
    }

    return all_ok;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(PAYLOAD_PIN, OUTPUT);
    digitalWrite(PAYLOAD_PIN, HIGH);

    Wire.begin();
    bool imu_ok = imuSensor.init();
    if (!imu_ok)
    {
        while (1)
        {
            flashLED(3);
        }
    }

    bool servo_ok = servoActuator.init();
    if (!servo_ok)
    {
        while (1)
        {
            flashLED(3);
        }
    }
    // servoActuator.setServoAngle(LEFT_WAIST_CH, waist_arr[0]);  // left waist
    // servoActuator.setServoAngle(RIGHT_WAIST_CH, waist_arr[1]); // right waist

    // CAN & ODrive
    if (!setupCan())
    {
        rclErrorLoop();
    }

    odrv_right_hip.onStatus(onHeartbeat, &D_right_hip);
    odrv_right_hip.onFeedback(onFeedback, &D_right_hip);
    odrv_left_hip.onStatus(onHeartbeat, &D_left_hip);
    odrv_left_hip.onFeedback(onFeedback, &D_left_hip);
    odrv_right_knee.onStatus(onHeartbeat, &D_right_knee);
    odrv_right_knee.onFeedback(onFeedback, &D_right_knee);
    odrv_left_knee.onStatus(onHeartbeat, &D_left_knee);
    odrv_left_knee.onFeedback(onFeedback, &D_left_knee);
    odrv_right_wheel.onStatus(onHeartbeat, &D_right_wheel);
    odrv_right_wheel.onFeedback(onFeedback, &D_right_wheel);
    odrv_left_wheel.onStatus(onHeartbeat, &D_left_wheel);
    odrv_left_wheel.onFeedback(onFeedback, &D_left_wheel);

    unsigned long t0 = millis();
    while (!(
        D_right_hip.got_hb &&
        D_left_hip.got_hb &&
        D_left_knee.got_hb &&
        D_right_knee.got_hb &&
        D_left_wheel.got_hb &&
        D_right_wheel.got_hb))
    {
        pumpEvents(can_intf);
        if (millis() - t0 > 2000)
            break; // ไม่บล็อกนานเกิน
    }

    bool zero_ok = capture_joint_zero(1200);

    odom.setFrames("odom", "base_link");
    odom.setGeometry((WHEEL_DIAMETER / 2.0f), LR_WHEELS_DISTANCE);
    odom.setUnwrapPosition(true); // ถ้า encoder wrap 2π
    odom.setOmegaLpfAlpha(0.05f);
    odom.setDtMax(1.0f);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    lqr_timer.begin(lqr_isr, 5000);
}

void loop()
{
    switch (connection_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, connection_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        connection_state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (connection_state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, connection_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (connection_state == AGENT_CONNECTED)
        {
            imuSensor.update();
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        }
        break;
    case AGENT_DISCONNECTED:
        lqr_enable = false;
        destroyEntities();
        connection_state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//

void twistCallback(const void *msgin)
{
    auto *m = (const geometry_msgs__msg__Twist *)msgin;
    twist_msg = *m;

    v_ref = twist_msg.linear.x;
    r_ref = twist_msg.angular.z;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0)
    {
        prev_cmd_time = millis();
    }
}

void leftLegCallback(const void *msgin)
{
    auto *m = (const geometry_msgs__msg__Twist *)msgin;
    left_leg_msg = *m; // ใช้ y=hip, z=knee ตามที่คุณ setPosition อยู่
    // left_leg_msg.y = left_leg_msg.y;
    // left_leg_msg.z = left_leg_msg.z;
}

void rightLegCallback(const void *msgin)
{
    auto *m = (const geometry_msgs__msg__Twist *)msgin;
    right_leg_msg = *m;
    // right_leg_msg.y = right_leg_msg.y;
    // right_leg_msg.z = right_leg_msg.z;
}

void robotCommandCallback(const void *msgin)
{
    auto *m = (const geometry_msgs__msg__Point *)msgin;
    robot_state_msg = *m;

    switch (int(robot_state_msg.x))
    {
    case IDLE:
        robot_state = IDLE;
        break;
    case SLIDE:
        robot_state = SLIDE;
        break;
    case OPERATING:
        robot_state = OPERATING;
        break;
    default:
        robot_state = IDLE;
        break;
    }

    switch (int(robot_state_msg.y))
    {
    case HOLD:
        payload_state = HOLD;
        break;
    case DEPLOY:
        payload_state = DEPLOY;
        break;
    default:
        payload_state = HOLD;
        break;
    }

    switch (int(robot_state_msg.z))
    {
    case TURN_ON:
        light_state = TURN_ON;
        break;
    case BLINK:
        light_state = BLINK;
        break;
    case TURN_OFF:
        light_state = TURN_OFF;
        break;
    default:
        light_state = TURN_ON;
        break;
    }
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        pumpEvents(can_intf);
        publishData();
        changeMode();
        moveBase();
        moveActuator();
    }
}

void changeMode()
{
    if (prev_robot_state != robot_state)
    {
        switch (robot_state)
        {
        case IDLE:
            lqr_enable = false;
            all_odrives_idle();
            wheels_set_velocity_mode();
            legs_set_position_mode();
            break;
        case SLIDE:
            lqr_enable = false;
            all_odrives_closed();
            wheels_set_velocity_mode();
            legs_set_position_mode();
            break;
        case OPERATING:
            lqr_enable = false;
            tau_L_cmd = 0;
            tau_R_cmd = 0;
            all_odrives_closed();
            wheels_set_torque_mode();
            legs_set_torque_mode();

            left_leg_msg.linear.y = joint_state_msg.position.data[J_LEFT_HIP];
            left_leg_msg.linear.z = joint_state_msg.position.data[J_LEFT_KNEE];
            right_leg_msg.linear.y = joint_state_msg.position.data[J_RIGHT_HIP];
            right_leg_msg.linear.z = joint_state_msg.position.data[J_RIGHT_KNEE];

            left_leg_msg.angular.y = 0.0f;
            left_leg_msg.angular.z = 0.0f;
            right_leg_msg.angular.y = 0.0f;
            right_leg_msg.angular.z = 0.0f;

            lqr_enable = true;
            break;
        default:
            lqr_enable = false;
            all_odrives_idle();
            wheels_set_velocity_mode();
            legs_set_position_mode();
            break;
        }
        prev_robot_state = robot_state;
    }
}

void moveBase()
{
    if (((millis() - prev_cmd_time) >= 500))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        noInterrupts();
        v_ref = 0.0f;
        r_ref = 0.0f;
        interrupts();
        digitalWrite(LED_PIN, HIGH);
    }

    if (robot_state == SLIDE)
    {
        Kinematics::rpm req_rpm = kinematics.getRPM(
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.angular.z);
        debug_msg.linear.x = req_rpm.motor1 / 60.f;
        debug_msg.linear.y = req_rpm.motor2 / 60.f;
        odrv_left_wheel.setVelocity(-req_rpm.motor1 / 60.f);
        odrv_right_wheel.setVelocity(req_rpm.motor2 / 60.f);
    }
    else if (robot_state == OPERATING)
    {
        float tauL = (float)tau_L_cmd;
        float tauR = (float)tau_R_cmd;

        // odrv_left_wheel.setTorque(-tauL);
        // odrv_right_wheel.setTorque(tauR);

        float qLh = joint_state_msg.position.data[J_LEFT_HIP];
        float qLk = joint_state_msg.position.data[J_LEFT_KNEE];
        float qRh = joint_state_msg.position.data[J_RIGHT_HIP];
        float qRk = joint_state_msg.position.data[J_RIGHT_KNEE];

        float qdLh = joint_state_msg.velocity.data[J_LEFT_HIP];
        float qdLk = joint_state_msg.velocity.data[J_LEFT_KNEE];
        float qdRh = joint_state_msg.velocity.data[J_RIGHT_HIP];
        float qdRk = joint_state_msg.velocity.data[J_RIGHT_KNEE];

        float qdesLh = left_leg_msg.linear.y;
        float qdesLk = left_leg_msg.linear.z;
        float qdesRh = right_leg_msg.linear.y;
        float qdesRk = right_leg_msg.linear.z;

        float qddesLh = left_leg_msg.angular.y;
        float qddesLk = left_leg_msg.angular.z;
        float qddesRh = right_leg_msg.angular.y;
        float qddesRk = right_leg_msg.angular.z;

        float tauLh = impedance_tau(qLh, qdLh, qdesLh, qddesLh, hip_g);
        float tauLk = impedance_tau(qLk, qdLk, qdesLk, qddesLk, knee_g);
        float tauRh = impedance_tau(qRh, qdRh, qdesRh, qddesRh, hip_g);
        float tauRk = impedance_tau(qRk, qdRk, qdesRk, qddesRk, knee_g);

        odrv_left_hip.setTorque(tauLh);
        odrv_left_knee.setTorque(-tauLk);
        odrv_right_hip.setTorque(-tauRh);
        odrv_right_knee.setTorque(tauRk);

        debug_msg.linear.x = qdesLh;
        debug_msg.linear.y = qdesRh;
        debug_msg.linear.z = tauLh;
        debug_msg.angular.x = tauLk;
        debug_msg.angular.y = tauRh;
        debug_msg.angular.z = tauRk;
    }
    else
    {
        odrv_left_wheel.setVelocity(0.0);
        odrv_right_wheel.setVelocity(0.0);
    }
}

void moveActuator()
{
    if (prev_payload_state != payload_state)
    {
        if (payload_state == DEPLOY)
        {
            payload_pulsing = true;
            payload_pulse_start_ms = millis();

            digitalWrite(PAYLOAD_PIN, LOW);
        }
        else
        {
            payload_pulsing = false;

            digitalWrite(PAYLOAD_PIN, HIGH);
        }
    }
    prev_payload_state = payload_state;

    if (payload_pulsing)
    {
        if (millis() - payload_pulse_start_ms >= PAYLOAD_PULSE_MS)
        {
            payload_pulsing = false;
            payload_state = HOLD;            // กลับเป็น HOLD
            digitalWrite(PAYLOAD_PIN, HIGH); // ปิดคอยล์
        }
    }

    if (prev_light_state != light_state)
    {
        switch (light_state)
        {
        case TURN_ON:
            break;
        case BLINK:
            break;
        case TURN_OFF:
            break;
        default:
            break;
        }
    }
    prev_light_state = light_state;
}

void publishData()
{
    got_left_wheel = false;
    got_right_wheel = false;

    struct timespec time_stamp = getTime();
    imu_msg = imuSensor.getData();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    joint_state_msg.position.data[J_LEFT_WAIST] = 0.0; // rad
    joint_state_msg.velocity.data[J_LEFT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_LEFT_WAIST] = 0.0;

    joint_state_msg.position.data[J_RIGHT_WAIST] = 0.0; // rad
    joint_state_msg.velocity.data[J_RIGHT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_RIGHT_WAIST] = 0.0;

    for (auto &m : maps)
    {
        Get_Encoder_Estimates_msg_t fb;
        // timeout เล็กๆ 2–5 ms พอ (อย่าใช้ 0 เผื่อเน็ตช้า)
        if (m.drv->getFeedback(fb, 3))
        {
            float pos_raw = fb.Pos_Estimate; // turns
            float vel_raw = fb.Vel_Estimate; // turns/s

            joint_state_motor_msg.position.data[m.pos_idx] = pos_raw;

            float pos_in = pos_raw;
            float vel_in = vel_raw;
            if (m.pos0_valid)
                pos_in = pos_raw - m.pos0_raw;

            if (m.flip)
            {
                pos_in = -pos_in;
                vel_in = -vel_in;
            }

            float pos_out = pos_in * m.pos_scale;
            float vel_out = vel_in * m.vel_scale;

            pos_out += m.pos_offset_rad;

            if (m.output_deg)
            {
                pos_out *= RAD2DEG;
                vel_out *= RAD2DEG;
            }

            const float s = dir_sign(m.direction);
            pos_out *= s;
            vel_out *= s;

            joint_state_msg.position.data[m.pos_idx] = pos_out;
            joint_state_msg.velocity.data[m.pos_idx] = vel_out;

            if (m.pos_idx == J_LEFT_WHEEL)
            {
                wheel_pos_l_rad = pos_out;
                got_left_wheel = true;
            }
            if (m.pos_idx == J_RIGHT_WHEEL)
            {
                wheel_pos_r_rad = pos_out;
                got_right_wheel = true;
            }
        }
    }
    have_wheel = got_left_wheel && got_right_wheel;

    uint32_t now_ms = millis();
    float dt = 0.0f;

    if (last_odom_ms == 0)
    {
        last_odom_ms = now_ms;
    }
    else
    {
        dt = (now_ms - last_odom_ms) * 1e-3f;
        last_odom_ms = now_ms;
    }

    bool odom_ok = true;
    if (dt <= 0.0f || dt > 0.1f)
    {
        noInterrupts();
        have_wheel_est = false;
        v_est = 0.0f;
        interrupts();
        odom_ok = false;
    }

    bool wheel_ok = have_wheel && odom_ok;

    noInterrupts();
    have_wheel_est = wheel_ok;
    interrupts();

    if (odom_ok && have_wheel)
    {
        bool ok = odom.updateFromWheelPos(wheel_pos_l_rad, wheel_pos_r_rad, dt);
        if (ok)
        {
            noInterrupts();
            v_est = odom.getVx();
            interrupts();
            odom_msg = odom.getData();
            odom_msg.header.stamp.sec = time_stamp.tv_sec;
            odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
            RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
        }
        else
        {
            noInterrupts();
            have_wheel_est = false;
            v_est = 0.0f;
            interrupts();
        }
    }

    // --------- FK: compute foot points ---------
    LegKinematics::joint_state jsL;
    jsL.waist = (float)joint_state_msg.position.data[J_LEFT_WAIST];
    jsL.hip = (float)joint_state_msg.position.data[J_LEFT_HIP];
    jsL.knee = (float)joint_state_msg.position.data[J_LEFT_KNEE];

    LegKinematics::joint_state jsR;
    jsR.waist = (float)joint_state_msg.position.data[J_RIGHT_WAIST];
    jsR.hip = (float)joint_state_msg.position.data[J_RIGHT_HIP];
    jsR.knee = (float)joint_state_msg.position.data[J_RIGHT_KNEE];

    LegKinematics::foot_point fpL = leg_left_fk.getFootPosition(jsL);
    LegKinematics::foot_point fpR = leg_right_fk.getFootPosition(jsR);

    left_foot_msg.x = fpL.x;
    left_foot_msg.y = fpL.y;
    left_foot_msg.z = fpL.z;

    right_foot_msg.x = fpR.x;
    right_foot_msg.y = fpR.y;
    right_foot_msg.z = fpR.z;

    if (robot_state == OPERATING && have_wheel_est)
    {
        float h_hat = -0.5f * (fpL.z + fpR.z);
        updateGainScheduling(h_hat);
    }

    RCSOFTCHECK(rcl_publish(&left_foot_publisher, &left_foot_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_foot_publisher, &right_foot_msg, NULL));

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
    RCSOFTCHECK(rcl_publish(&joint_state_motor_publisher, &joint_state_motor_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "main_teensy", "", &support));
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states/hardware"));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states/motor"));
    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/debug/cmd_vel"));
    RCCHECK(rclc_publisher_init_default(
        &left_foot_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "/ambula/left_foot_point"));
    RCCHECK(rclc_publisher_init_default(
        &right_foot_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "/ambula/right_foot_point"));

    // Allocate JointState sequences
    rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.velocity, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.effort, N_JOINTS);
    for (size_t i = 0; i < N_JOINTS; ++i)
    {
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[i], JOINT_NAMES[i]);
        joint_state_msg.position.data[i] = 0.0;
        joint_state_msg.velocity.data[i] = 0.0;
        joint_state_msg.effort.data[i] = 0.0;
    }

    rosidl_runtime_c__String__init(&joint_state_motor_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&joint_state_motor_msg.header.frame_id, "base_link");
    rosidl_runtime_c__String__Sequence__init(&joint_state_motor_msg.name, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_motor_msg.position, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_motor_msg.velocity, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_state_motor_msg.effort, N_JOINTS);
    for (size_t i = 0; i < N_JOINTS; ++i)
    {
        rosidl_runtime_c__String__assign(&joint_state_motor_msg.name.data[i], JOINT_NAMES[i]);
        joint_state_motor_msg.position.data[i] = 0.0;
        joint_state_motor_msg.velocity.data[i] = 0.0;
        joint_state_motor_msg.effort.data[i] = 0.0;
    }

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(
        &left_leg_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_joint/left"));
    RCCHECK(rclc_subscription_init_default(
        &right_leg_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_joint/right"));
    RCCHECK(rclc_subscription_init_default(
        &robot_state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "robot/state"));

    // create timer for actuating the motors at 200 Hz (1000/5 ms)
    const unsigned int control_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &left_leg_subscriber,
        &left_leg_msg,
        &leftLegCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &right_leg_subscriber,
        &right_leg_msg,
        &rightLegCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &robot_state_subscriber,
        &robot_state_msg,
        &robotCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // servoActuator.setServoAngle(LEFT_WAIST_CH, 90.0f);  // left waist
    // servoActuator.setServoAngle(RIGHT_WAIST_CH, 90.0f); // right waist

    rosidl_runtime_c__String__fini(&joint_state_msg.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_state_msg.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.velocity);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.effort);

    rosidl_runtime_c__String__fini(&joint_state_motor_msg.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_state_motor_msg.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_motor_msg.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_motor_msg.velocity);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_motor_msg.effort);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_publisher_fini(&joint_state_motor_publisher, &node);
    rcl_publisher_fini(&left_foot_publisher, &node);
    rcl_publisher_fini(&right_foot_publisher, &node);
    rcl_publisher_fini(&debug_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&left_leg_subscriber, &node);
    rcl_subscription_fini(&right_leg_subscriber, &node);
    rcl_subscription_fini(&robot_state_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

bool request_state_with_retry(
    ODriveCAN &drv, ODrvData &ud, ODriveAxisState desired_state,
    uint32_t settle_ms = 150, uint32_t per_try_timeout_ms = 400, int max_retry = 5)
{
    for (int k = 0; k < max_retry; ++k)
    {
        drv.clearErrors();           // ถ้าไม่อยาก clear ก็ลบทิ้งบรรทัดนี้ได้
        drv.setState(desired_state); // ← ตอนนี้ชนิดตรงกับ API แล้ว

        uint32_t t0 = millis();
        while (millis() - t0 < settle_ms)
        {
            delay(10);
            pumpEvents(can_intf);
        }
        t0 = millis();
        while (millis() - t0 < per_try_timeout_ms)
        {
            pumpEvents(can_intf);
            if (ud.got_hb && ud.hb.Axis_State == desired_state)
                return true;
            delay(5);
        }
    }
    return false;
}

void all_odrives_idle()
{
    odrv_right_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_left_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_right_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_left_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_right_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_left_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
}

void all_odrives_closed()
{
    bool ok_rh = request_state_with_retry(
        odrv_right_hip, D_right_hip,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    bool ok_lh = request_state_with_retry(
        odrv_left_hip, D_left_hip,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    bool ok_rk = request_state_with_retry(
        odrv_right_knee, D_right_knee,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    bool ok_lk = request_state_with_retry(
        odrv_left_knee, D_left_knee,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    bool ok_rw = request_state_with_retry(
        odrv_right_wheel, D_right_wheel,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    bool ok_lw = request_state_with_retry(
        odrv_left_wheel, D_left_wheel,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    if (!ok_rh)
        odrv_right_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_lh)
        odrv_left_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_rk)
        odrv_right_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_lk)
        odrv_left_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_rw)
        odrv_right_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_lw)
        odrv_left_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
}

void wheels_set_velocity_mode()
{
    // สั่งให้ ODrive "ล้อ" เข้า VELOCITY_CONTROL + PASSTHROUGH
    // (บางไลบรารีเรียก INPUT_MODE_PASSTHROUGH หรือ INPUT_MODE_PASSTHROUGH = 1)
    odrv_left_wheel.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_wheel.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);

    // กันกระชากตอนสลับโหมด: ส่ง setpoint 0 หนึ่งครั้ง
    odrv_left_wheel.setVelocity(0.0f);
    odrv_right_wheel.setVelocity(0.0f);
}

void wheels_set_torque_mode()
{
    // สั่งให้ ODrive "ล้อ" เข้า TORQUE_CONTROL + PASSTHROUGH
    odrv_left_wheel.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_wheel.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);

    // กันกระชาก: ส่ง torque 0 หนึ่งครั้ง
    odrv_left_wheel.setTorque(0.0f);
    odrv_right_wheel.setTorque(0.0f);
}

void legs_set_position_mode()
{
    odrv_left_hip.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_left_knee.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_hip.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_knee.setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
}

void legs_set_torque_mode()
{
    odrv_left_hip.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_left_knee.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_hip.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_knee.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);

    // กันกระชาก
    odrv_left_hip.setTorque(0.0f);
    odrv_left_knee.setTorque(0.0f);
    odrv_right_hip.setTorque(0.0f);
    odrv_right_knee.setTorque(0.0f);
}
