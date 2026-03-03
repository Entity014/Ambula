#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <IntervalTimer.h>

#include "config.h"
#include "lqr.h" // <--- เพิ่ม LQR Header
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
rcl_publisher_t odom_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t joint_cmd_subscriber;
rcl_subscription_t robot_state_subscriber;
rcl_subscription_t payload_state_subscriber;
rcl_subscription_t led_state_subscriber;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState joint_state_motor_msg;
sensor_msgs__msg__JointState joint_cmd_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int8 robot_state_msg;
std_msgs__msg__Int8 payload_state_msg;
std_msgs__msg__Int8 led_state_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist debug_msg;

rclc_executor_t executor_control, executor_feedback;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node_control;
rcl_timer_t control_timer, feedback_timer;
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

// ---------- Leg Kinematics สำหรับประมาณความสูงให้ LQR ----------
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

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

static inline float dir_sign(int8_t d)
{
    return (d >= 0) ? 1.0f : -1.0f;
}

float target_joint_pos[N_JOINTS] = {0.0f};
float target_joint_eff[N_JOINTS] = {0.0f};

//------------------------------ < LQR Components > ------------------------------//
static IntervalTimer lqr_timer;

// shared outputs from ISR
static volatile float tau_L_cmd = 0.0f;
static volatile float tau_R_cmd = 0.0f;

// shared references from ROS callback
static volatile float v_ref_lqr = 0.0f;
static volatile float r_ref_lqr = 0.0f;

// enable flag
static volatile bool lqr_enable = false;

static volatile float v_est = 0.0f;
static volatile float v_est_left = 0.0f;
static volatile float v_est_right = 0.0f;
static volatile bool have_wheel_est = false;

static volatile float pitch_ref_lqr = 0.0f;

// ----- Outer PI Velocity-to-Pitch -----
static float vel_integral = 0.0f;
static const float Kp_v2p = 0.15f;  // เริ่มต้นปลอดภัย
static const float Ki_v2p = 0.011f; // เริ่มต้นปลอดภัย
static const float I_max = 0.6f;

struct LqrGains
{
    float k00, k01, k02, k03; // tau_fwd
    float k10, k11, k12, k13; // tau_yaw
};

// ----- Scheduled gains shared with ISR -----
static volatile LqrGains K_sched = {
    -3.635093f, 0.311460f, -0.729928f, 0.000000f,
    0.0f, 0.0f, 0.0f, 0.100000f};

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

void lqr_isr()
{
    if (!lqr_enable)
    {
        tau_L_cmd = tau_R_cmd = 0.0f;
        return;
    }

    float theta, theta_dot, r_imu;
    LqrGains Klocal;
    float vl = 0.0f, vr = 0.0f; // ← เก็บค่าที่ปลอดภัย
    float vref = 0.0f, rref = 0.0f;
    float pitch_ref = 0.0f;

    noInterrupts();
    theta = imuSensor.getPitch();
    theta_dot = imuSensor.getPitchRate();
    r_imu = imuSensor.getYawRate();
    vl = v_est_left; // copy มาใช้
    vr = v_est_right;
    vref = v_ref_lqr;
    rref = r_ref_lqr;
    pitch_ref = pitch_ref_lqr;
    Klocal.k00 = K_sched.k00;
    Klocal.k01 = K_sched.k01;
    Klocal.k02 = K_sched.k02;
    Klocal.k03 = K_sched.k03;
    Klocal.k10 = K_sched.k10;
    Klocal.k11 = K_sched.k11;
    Klocal.k12 = K_sched.k12;
    Klocal.k13 = K_sched.k13;
    interrupts();

    if (fabsf(theta) > 0.7f)
    {
        tau_L_cmd = tau_R_cmd = 0.0f;
        return;
    }

    float x0 = (theta - pitch_ref);
    float x1 = theta_dot;
    float x2 = (v_est - vref); // ใช้ v_est หลัก (จาก odometry)
    float x3 = (r_imu - rref);

    float tau_fwd = -(Klocal.k00 * x0 + Klocal.k01 * x1 + Klocal.k02 * x2 + Klocal.k03 * x3);
    float tau_yaw = -(Klocal.k10 * x0 + Klocal.k11 * x1 + Klocal.k12 * x2 + Klocal.k13 * x3);

    float tauL = tau_fwd - tau_yaw;
    float tauR = tau_fwd + tau_yaw;

    // ================== Slip Compensation (แก้ vR_isr → vr) ==================
    float diff_vel = (vr - vl) - (rref * LR_WHEELS_DISTANCE); // ← แก้ตรงนี้
    float K_slip = 1.0f;                                      // จูนได้
    float tau_slip = K_slip * diff_vel;

    // tauL += tau_slip; // ล้อซ้าย +
    // tauR -= tau_slip; // ล้อขวา -

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

    float pos_scale;
    float vel_scale;
    float pos_offset_rad;
    bool flip;
    int8_t direction;
    bool pos0_valid;
    float pos0_raw;
    bool output_deg;
};

JointCfg maps[] = {
    {&odrv_left_hip, &D_left_hip, J_LEFT_HIP, 2.0f * M_PI / 20.0f, 2.0f * M_PI / 20.0f, 65.0f * DEG2RAD, true, +1, false, 0.0f, false},
    {&odrv_left_knee, &D_left_knee, J_LEFT_KNEE, 2.0f * M_PI / 10.0f, 2.0f * M_PI / 10.0f, 140.0f * DEG2RAD, false, +1, false, 0.0f, false},
    {&odrv_left_wheel, &D_left_wheel, J_LEFT_WHEEL, 2.0f * M_PI * 0.67f, 2.0f * M_PI * 0.67f, 0.0f, true, +1, false, 0.0f, false},
    {&odrv_right_hip, &D_right_hip, J_RIGHT_HIP, 2.0f * M_PI / 20.0f, 2.0f * M_PI / 20.0f, 65.0f * DEG2RAD, false, +1, false, 0.0f, false},
    {&odrv_right_knee, &D_right_knee, J_RIGHT_KNEE, 2.0f * M_PI / 10.0f, 2.0f * M_PI / 10.0f, 140.0f * DEG2RAD, true, +1, false, 0.0f, false},
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
            if (m.drv->getFeedback(fb, 5))
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

    bool zero_ok = capture_joint_zero(2000);

    odom.setFrames("odom", "base_link");
    odom.setGeometry((WHEEL_DIAMETER / 2.0f), LR_WHEELS_DISTANCE);
    odom.setUnwrapPosition(true);
    odom.setOmegaLpfAlpha(0.05f);
    odom.setDtMax(1.0f);

    Serial.begin(921600);
    set_microros_serial_transports(Serial);

    // เปิด LQR Timer ทิ้งไว้ที่ 200 Hz (5000 us)
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
            rclc_executor_spin_some(&executor_control, RCL_MS_TO_NS(10));
            rclc_executor_spin_some(&executor_feedback, RCL_MS_TO_NS(10));
        }
        break;
    case AGENT_DISCONNECTED:
        lqr_enable = false; // ปิด LQR เซฟหุ่นตอนหลุดการเชื่อมต่อ
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

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0)
    {
        prev_cmd_time = millis();
    }

    // อัปเดต Reference ให้ LQR ISR
    noInterrupts();
    v_ref_lqr = twist_msg.linear.x;
    r_ref_lqr = twist_msg.angular.z;
    interrupts();
}

void jointCommandCallback(const void *msgin)
{
    auto *m = (const sensor_msgs__msg__JointState *)msgin;

    // วนลูปเช็คชื่อ Joint และเก็บค่าให้ตรง Index
    for (size_t i = 0; i < m->position.size; i++)
    {
        // หา Index ที่ตรงกับ JointIndex enum ของเรา
        for (int j = 0; j < N_JOINTS; j++)
        {
            if (strcmp(m->name.data[i].data, JOINT_NAMES[j]) == 0)
            {
                target_joint_pos[j] = m->position.data[i];
                target_joint_eff[j] = m->effort.data[i];
                break;
            }
        }
    }
}
void robotCommandCallback(const void *msgin)
{
    auto *m = (const std_msgs__msg__Int8 *)msgin;
    robot_state_msg = *m;

    switch (int(robot_state_msg.data))
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
}

void payloadCommandCallback(const void *msgin)
{
    auto *m = (const std_msgs__msg__Int8 *)msgin;
    payload_state_msg = *m;

    switch (payload_state_msg.data)
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
}

void ledCommandCallback(const void *msgin)
{
    auto *m = (const std_msgs__msg__Int8 *)msgin;
    led_state_msg = *m;

    switch (led_state_msg.data)
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
        changeMode();
        moveBase();
        moveActuator();

        if (robot_state == OPERATING && have_wheel_est)
        {
            float v_err = v_ref_lqr - v_est;

            vel_integral += v_err * 0.002f;                     // dt = 5 ms (200 Hz)
            vel_integral = clampf(vel_integral, -I_max, I_max); // anti-windup

            float pitch_cmd = Kp_v2p * v_err + Ki_v2p * vel_integral;

            noInterrupts();
            pitch_ref_lqr = clampf(pitch_cmd, -0.4f, 0.4f);
            interrupts();
        }
        else
        {
            noInterrupts();
            pitch_ref_lqr = 0.0f;
            vel_integral = 0.0f; // reset integral
            interrupts();
        }

        struct timespec time_stamp = getTime();
        imu_msg = imuSensor.getData();
        imu_msg.header.stamp.sec = time_stamp.tv_sec;
        imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}

void feedbackCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        pumpEvents(can_intf);
        publishData();
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
            vel_integral = 0.0f;
            all_odrives_idle();
            wheels_set_velocity_mode();
            break;
        case SLIDE:
            lqr_enable = false;
            vel_integral = 0.0f;
            all_odrives_closed();
            wheels_set_velocity_mode();
            break;
        case OPERATING:
            noInterrupts();
            vel_integral = 0.0f;  // ล้างค่า Integral สะสมจากโหมดอื่น
            pitch_ref_lqr = 0.0f; // เริ่มจากตัวตรงเสมอ
            v_ref_lqr = 0.0f;
            r_ref_lqr = 0.0f;
            tau_L_cmd = 0.0f;
            tau_R_cmd = 0.0f;
            interrupts();

            for (int i = 0; i < N_JOINTS; i++)
            {
                target_joint_eff[i] = 0.0f;
            }
            tau_L_cmd = 0;
            tau_R_cmd = 0;
            target_joint_pos[J_LEFT_HIP] = 1.57f;
            target_joint_pos[J_LEFT_KNEE] = 2.408554368f;
            target_joint_pos[J_RIGHT_HIP] = 1.57f;
            target_joint_pos[J_RIGHT_KNEE] = 2.408554368f;
            all_odrives_closed();
            wheels_set_torque_mode();
            lqr_enable = true; // เปิด LQR ในโหมดทรงตัว
            break;
        default:
            lqr_enable = false;
            vel_integral = 0.0f;
            all_odrives_idle();
            wheels_set_velocity_mode();
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
        v_ref_lqr = 0.0f;
        r_ref_lqr = 0.0f;
        interrupts();

        digitalWrite(LED_PIN, HIGH);
    }

    if (robot_state == SLIDE)
    {
        float left_hip_turns = ((2.0 * dir_sign(maps[0].direction)) - maps[0].pos_offset_rad) / maps[0].pos_scale;
        if (maps[0].flip)
            left_hip_turns = -left_hip_turns;
        left_hip_turns += maps[0].pos0_raw;
        odrv_left_hip.setPosition(left_hip_turns, 0.0f, 0.0f);

        float left_knee_turns = ((2.408554368 * dir_sign(maps[1].direction)) - maps[1].pos_offset_rad) / maps[1].pos_scale;
        if (maps[1].flip)
            left_knee_turns = -left_knee_turns;
        left_knee_turns += maps[1].pos0_raw;
        odrv_left_knee.setPosition(left_knee_turns, 0.0f, 0.0f);

        float right_hip_turns = ((2.0 * dir_sign(maps[3].direction)) - maps[3].pos_offset_rad) / maps[3].pos_scale;
        if (maps[3].flip)
            right_hip_turns = -right_hip_turns;
        right_hip_turns += maps[3].pos0_raw;
        odrv_right_hip.setPosition(right_hip_turns, 0.0f, 0.0f);

        float right_knee_turns = ((2.408554368 * dir_sign(maps[4].direction)) - maps[4].pos_offset_rad) / maps[4].pos_scale;
        if (maps[4].flip)
            right_knee_turns = -right_knee_turns;
        right_knee_turns += maps[4].pos0_raw;
        odrv_right_knee.setPosition(right_knee_turns, 0.0f, 0.0f);

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
        // 1. นำแรงบิดจาก LQR (ที่วิ่งอยู่เบื้องหลัง) มาจ่ายให้ล้อ
        float tau_wheel_left = (float)tau_L_cmd;
        float tau_wheel_right = (float)tau_R_cmd;
        odrv_left_wheel.setTorque(-tau_wheel_left);
        odrv_right_wheel.setTorque(tau_wheel_right);

        // 2. รับค่าจาก WBC (ผ่าน ROS) ไปคุม Position และ Torque ของขา
        float left_hip_turns = ((target_joint_pos[J_LEFT_HIP] * dir_sign(maps[0].direction)) - maps[0].pos_offset_rad) / maps[0].pos_scale;
        if (maps[0].flip)
            left_hip_turns = -left_hip_turns;
        left_hip_turns += maps[0].pos0_raw;
        odrv_left_hip.setPosition(left_hip_turns, 0.0f, -target_joint_eff[J_LEFT_HIP] * 0.05f);

        float left_knee_turns = ((target_joint_pos[J_LEFT_KNEE] * dir_sign(maps[1].direction)) - maps[1].pos_offset_rad) / maps[1].pos_scale;
        if (maps[1].flip)
            left_knee_turns = -left_knee_turns;
        left_knee_turns += maps[1].pos0_raw;
        odrv_left_knee.setPosition(left_knee_turns, 0.0f, target_joint_eff[J_LEFT_KNEE] * 0.1f);

        float right_hip_turns = ((target_joint_pos[J_RIGHT_HIP] * dir_sign(maps[3].direction)) - maps[3].pos_offset_rad) / maps[3].pos_scale;
        if (maps[3].flip)
            right_hip_turns = -right_hip_turns;
        right_hip_turns += maps[3].pos0_raw;
        odrv_right_hip.setPosition(right_hip_turns, 0.0f, target_joint_eff[J_RIGHT_HIP] * 0.05f);

        float right_knee_turns = ((target_joint_pos[J_RIGHT_KNEE] * dir_sign(maps[4].direction)) - maps[4].pos_offset_rad) / maps[4].pos_scale;
        if (maps[4].flip)
            right_knee_turns = -right_knee_turns;
        right_knee_turns += maps[4].pos0_raw;
        odrv_right_knee.setPosition(right_knee_turns, 0.0f, -target_joint_eff[J_RIGHT_KNEE] * 0.1f);

        debug_msg.linear.x = -tau_wheel_left;
        debug_msg.linear.y = tau_wheel_right;
        debug_msg.linear.z = left_hip_turns;
        debug_msg.angular.x = left_knee_turns;
        debug_msg.angular.y = right_hip_turns;
        debug_msg.angular.z = right_knee_turns;
    }
    else
    {
        odrv_left_wheel.setVelocity(0.0);
        odrv_right_wheel.setVelocity(0.0);
    }
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
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
            payload_state = HOLD;
            digitalWrite(PAYLOAD_PIN, HIGH);
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
    struct timespec time_stamp = getTime();
    joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    got_left_wheel = false;
    got_right_wheel = false;

    for (auto &m : maps)
    {
        Get_Encoder_Estimates_msg_t fb;
        if (m.drv->getFeedback(fb, 5))
        {
            float pos_raw = fb.Pos_Estimate;
            float vel_raw = fb.Vel_Estimate;

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
    float dt = (last_odom_ms == 0) ? 0.0f : (now_ms - last_odom_ms) * 1e-3f;
    last_odom_ms = now_ms;

    bool odom_ok = true;
    if (dt <= 0.0f || dt > 0.1f)
    {
        noInterrupts();
        have_wheel_est = false;
        v_est = v_est_left = v_est_right = 0.0f;
        interrupts();
        odom_ok = false;
    }

    bool wheel_ok = have_wheel && odom_ok;

    noInterrupts();
    have_wheel_est = wheel_ok;
    interrupts();

    if (odom_ok && have_wheel)
    {
        if (odom.updateFromWheelPos(wheel_pos_l_rad, wheel_pos_r_rad, dt))
        {
            // ✅ เรียกฟังก์ชันก่อน แล้วค่อย copy เข้า volatile ภายใต้ critical section
            float temp_left = 0.0f, temp_right = 0.0f;
            float vx = odom.getVx();

            odom.getWheelVelocities(temp_left, temp_right);

            noInterrupts();
            v_est = vx;
            v_est_left = temp_left;
            v_est_right = temp_right;
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
            v_est = v_est_left = v_est_right = 0.0f;
            interrupts();
        }
    }

    // --------- คำนวณ FK เพื่ออัปเดต Gain Scheduling ของ LQR ---------
    LegKinematics::joint_state jsL;
    jsL.waist = 0.0f;
    jsL.hip = -(float)joint_state_msg.position.data[J_LEFT_HIP];
    jsL.knee = -(float)joint_state_msg.position.data[J_LEFT_KNEE];

    LegKinematics::joint_state jsR;
    jsR.waist = 0.0f;
    jsR.hip = -(float)joint_state_msg.position.data[J_RIGHT_HIP];
    jsR.knee = -(float)joint_state_msg.position.data[J_RIGHT_KNEE];

    LegKinematics::foot_point fpL = leg_left_fk.getFootPosition(jsL);
    LegKinematics::foot_point fpR = leg_right_fk.getFootPosition(jsR);

    // ถ้า LQR เปิดอยู่ ให้ประเมินความสูงแล้วส่งไปปรับ Gain
    if (robot_state == OPERATING && have_wheel_est)
    {
        float h_hat = -0.5f * (fpL.z + fpR.z);
        updateGainScheduling(h_hat);
    }

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
    RCSOFTCHECK(rcl_publish(&joint_state_motor_publisher, &joint_state_motor_msg, NULL));
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node_control, "teensy_control", "", &support));
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states/hardware"));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_motor_publisher,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states/motor"));
    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/cmd_vel"));

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
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(
        &joint_cmd_subscriber,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "cmd_joint"));
    RCCHECK(rclc_subscription_init_default(
        &robot_state_subscriber,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "robot/state"));
    RCCHECK(rclc_subscription_init_default(
        &payload_state_subscriber,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "payload/state"));
    RCCHECK(rclc_subscription_init_default(
        &led_state_subscriber,
        &node_control,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "led/state"));

    rosidl_runtime_c__String__init(&joint_cmd_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&joint_cmd_msg.header.frame_id, "base_link");
    rosidl_runtime_c__String__Sequence__init(&joint_cmd_msg.name, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_cmd_msg.position, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_cmd_msg.velocity, N_JOINTS);
    rosidl_runtime_c__double__Sequence__init(&joint_cmd_msg.effort, N_JOINTS);
    for (size_t i = 0; i < N_JOINTS; ++i)
    {
        rosidl_runtime_c__String__assign(&joint_cmd_msg.name.data[i], JOINT_NAMES[i]);
        joint_cmd_msg.position.data[i] = 0.0;
        joint_cmd_msg.velocity.data[i] = 0.0;
        joint_cmd_msg.effort.data[i] = 0.0;
    }

    // create timer for actuating the motors at 200 Hz (1000/5 ms)
    const unsigned int control_timeout = 2, feedback_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    RCCHECK(rclc_timer_init_default(
        &feedback_timer,
        &support,
        RCL_MS_TO_NS(feedback_timeout),
        feedbackCallback));
    executor_control = rclc_executor_get_zero_initialized_executor();
    executor_feedback = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_control, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_init(&executor_feedback, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor_control,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor_control,
        &joint_cmd_subscriber,
        &joint_cmd_msg,
        &jointCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor_control,
        &robot_state_subscriber,
        &robot_state_msg,
        &robotCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor_control,
        &payload_state_subscriber,
        &payload_state_msg,
        &payloadCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor_control,
        &led_state_subscriber,
        &led_state_msg,
        &ledCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor_control, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor_feedback, &feedback_timer));

    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    digitalWrite(PAYLOAD_PIN, HIGH);

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

    rosidl_runtime_c__String__fini(&joint_cmd_msg.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_cmd_msg.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_cmd_msg.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_cmd_msg.velocity);
    rosidl_runtime_c__double__Sequence__fini(&joint_cmd_msg.effort);

    rcl_publisher_fini(&odom_publisher, &node_control);
    rcl_publisher_fini(&imu_publisher, &node_control);
    rcl_publisher_fini(&joint_state_publisher, &node_control);
    rcl_publisher_fini(&joint_state_motor_publisher, &node_control);
    rcl_publisher_fini(&debug_publisher, &node_control);
    rcl_subscription_fini(&twist_subscriber, &node_control);
    rcl_subscription_fini(&joint_cmd_subscriber, &node_control);
    rcl_subscription_fini(&robot_state_subscriber, &node_control);
    rcl_subscription_fini(&payload_state_subscriber, &node_control);
    rcl_subscription_fini(&led_state_subscriber, &node_control);
    rcl_node_fini(&node_control);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&feedback_timer);
    rclc_executor_fini(&executor_control);
    rclc_executor_fini(&executor_feedback);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
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
        drv.clearErrors();
        drv.setState(desired_state);

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
    odrv_left_wheel.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_wheel.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_left_wheel.setVelocity(0.0f);
    odrv_right_wheel.setVelocity(0.0f);
}

void wheels_set_torque_mode()
{
    odrv_left_wheel.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_right_wheel.setControllerMode(CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);
    odrv_left_wheel.setTorque(0.0f);
    odrv_right_wheel.setTorque(0.0f);
}