#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// ---------- ROS 2 / micro-ROS ----------
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <micro_ros_utilities/type_utilities.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>

// ---------- ODrive CAN ----------
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // teensy compile hack

// ======================== Config =========================
#define ROS_DOMAIN_ID 15
#define CAN_BAUDRATE 250000

// Joint naming & order
#define N_JOINTS 8
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
const char *JOINT_NAMES[N_JOINTS] = {
    "left_waist", "left_hip", "left_knee", "left_wheel",
    "right_waist", "right_hip", "right_knee", "right_wheel"};

// ODrive Node IDs (ตามที่ผู้ใช้ระบุ)
#define ODRV_RIGHT_HIP_ID 0   // Right Hip
#define ODRV_LEFT_HIP_ID 2    // Left Hip
#define ODRV_RIGHT_KNEE_ID 3  // Right Knee
#define ODRV_LEFT_KNEE_ID 5   // Left Knee
#define ODRV_RIGHT_WHEEL_ID 4 // Right Wheel
#define ODRV_LEFT_WHEEL_ID 6  // Left Wheel

// Servo PWM (waist)
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50
#define LEFT_WAIST_CH 9
#define RIGHT_WAIST_CH 8
#define LED_CH 4

// LEDs / pins (optional)
#define LED_PIN 13
#define ESTOP_PIN 9
#define PAYLOAD_PIN 26

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
rcl_publisher_t odom_publisher;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t battery_publisher;
rcl_subscription_t joint_command_subscriber;
rcl_subscription_t robot_command_subscriber;
rcl_subscription_t led_command_subscriber;
rcl_subscription_t payload_command_subscriber;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__JointState joint_command_msg;
std_msgs__msg__Int8 robot_command_msg;
std_msgs__msg__Int8 led_command_msg;
std_msgs__msg__Bool payload_command_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

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

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void moveBase();
void moveArm();
void publish_joint_state();
void publishData();
bool createEntities();
bool destroyEntities();
void tcaSelect(uint8_t i);
struct timespec getTime();
void setServoAngle(uint8_t channel, float angle);
void setWaistAnglesDeg(float left_deg, float right_deg);
void all_odrives_idle();
void wheels_closed_loop_only_safe();
int find_index_in_msg(const sensor_msgs__msg__JointState *m, const char *target);

//------------------------------ < Rapid Define > -------------------------------------//

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float waist_cmd_deg_left = 90.f;
float waist_cmd_deg_right = 90.f;

float g_cmd_left_radps = 0.0f;
float g_cmd_right_radps = 0.0f;
unsigned long g_last_cmd_ms = 0;

const float RAD2TURNS = 1.0f / (2.0f * M_PI); // 1 turn = 2π rad
const float VEL_MAX_TURNS = 10.0f;            // ปรับตามสเปค/ความปลอดภัย
const uint32_t CMD_TIMEOUT_MS = 300;          // เกินนี้จะหยุดล้อ

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

// ---------- IMU (BNO08x) ----------
BNO08x myIMU;

#define BNO08X_INT -1    // ใช้ -1 ถ้าไม่ต่อขา INT
#define BNO08X_RST -1    // ใช้ -1 ถ้าไม่ต่อขา RST
#define BNO08X_ADDR 0x4A // ถ้า ADR jumper ปิด = 0x4A (ตามตัวอย่างของไฟต์)

bool imu_ready = false;

void imuSetReports(); // proto สำหรับตั้งค่า report
void publish_imu();   // proto สำหรับฟังก์ชัน publish IMU

//------------------------------ < Main > -------------------------------------//

void setup()
{
    robot_state = IDLE;

    pinMode(LED_PIN, OUTPUT);
    pinMode(ESTOP_PIN, OUTPUT);
    // pinMode(PAYLOAD_PIN, OUTPUT);
    // digitalWrite(PAYLOAD_PIN, HIGH);

    Wire.begin();

    if (!myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST))
    {
        // ถ้าไม่เจอ IMU: กระพริบไฟเร็ว ๆ แล้วไม่ตั้ง imu_ready
        for (int i = 0; i < 5; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
        imu_ready = false;
    }
    else
    {
        imu_ready = true;
        imuSetReports(); // เปิด Geomagnetic Rotation Vector
    }

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // เซอร์โวทั่วไปใช้ 50Hz

    setWaistAnglesDeg(90.f, 90.f);

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
        D_right_hip.got_hb ||
        D_left_hip.got_hb ||
        D_left_knee.got_hb ||
        D_right_knee.got_hb ||
        D_left_wheel.got_hb ||
        D_right_wheel.got_hb))
    {
        pumpEvents(can_intf);
        if (millis() - t0 > 2000)
            break; // ไม่บล็อกนานเกิน
    }

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
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
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        connection_state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        moveBase();
        moveArm();
        publishData();
    }
}

void JointCommandCallback(const void *msgin)
{
    const auto *m = (const sensor_msgs__msg__JointState *)msgin;
    g_last_cmd_ms = millis();

    // ตรวจสอบขนาด array เพื่อป้องกัน out-of-range
    if (m->velocity.size >= 8)
    {
        g_cmd_left_radps = (float)m->velocity.data[3];  // left_wheel
        g_cmd_right_radps = (float)m->velocity.data[7]; // right_wheel
    }
    else
    {
        g_cmd_left_radps = 0.0f;
        g_cmd_right_radps = 0.0f;
    }
}

void RobotCommandCallback(const void *msgin)
{
    auto *m = (const std_msgs__msg__Int8 *)msgin;
    switch (m->data)
    {
    case IDLE:
        robot_state = IDLE;
        all_odrives_idle();
        break;
    case SLIDE:
        robot_state = SLIDE;
        wheels_closed_loop_only_safe();
        break;
    case OPERATING:
        robot_state = OPERATING; /* TODO: โหมดควบคุมหลัก */
        break;
    default:
        robot_state = IDLE;
        all_odrives_idle();
        break;
    }
}

void LEDCommandCallback(const void *msgin)
{
}

void PayloadCommandCallback(const void *msgin)
{
}

static micro_ros_utilities_memory_conf_t g_joint_cmd_conf = {
    .max_string_capacity = 32,
    .max_ros2_type_sequence_capacity = 8,
    .max_basic_type_sequence_capacity = 8,
    .rules = NULL,
    .n_rules = 0,
    .allocator = NULL};

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

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

    sensor_msgs__msg__JointState__init(&joint_command_msg);

    bool ok = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_command_msg,
        g_joint_cmd_conf);
    if (!ok)
    {
        rclErrorLoop();
    }

    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    // กำหนด covariance:
    // orientation: ค่า 0 หมายถึงรู้ค่าครบ แต่เราไม่ได้ใส่ค่า covariance จริง ๆ -> ถ้าจะบอกว่าไม่รู้ให้ใช้ -1
    for (int i = 0; i < 9; i++)
    {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = -1.0; // -1 = ไม่ให้ใช้ค่า
        imu_msg.linear_acceleration_covariance[i] = -1.0;
    }

    // create node
    RCCHECK(rclc_node_init_default(&node, "main_teensy", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states/hardware"));
    RCCHECK(rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery/state"));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &joint_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states/command"));
    RCCHECK(rclc_subscription_init_default(
        &robot_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "robot/state"));
    RCCHECK(rclc_subscription_init_default(
        &led_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "led/state"));
    RCCHECK(rclc_subscription_init_default(
        &payload_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "payload/command"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &joint_command_subscriber,
        &joint_command_msg,
        &JointCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &robot_command_subscriber,
        &robot_command_msg,
        &RobotCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &led_command_subscriber,
        &led_command_msg,
        &LEDCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &payload_command_subscriber,
        &payload_command_msg,
        &PayloadCommandCallback,
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

    setWaistAnglesDeg(90.f, 90.f);

    rosidl_runtime_c__String__fini(&joint_state_msg.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_state_msg.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.velocity);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.effort);

    micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_command_msg,
        g_joint_cmd_conf);
    sensor_msgs__msg__JointState__fini(&joint_command_msg);

    // Free IMU msg
    rosidl_runtime_c__String__fini(&imu_msg.header.frame_id);
    sensor_msgs__msg__Imu__fini(&imu_msg);

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_publisher_fini(&battery_publisher, &node);
    rcl_subscription_fini(&joint_command_subscriber, &node);
    rcl_subscription_fini(&robot_command_subscriber, &node);
    rcl_subscription_fini(&led_command_subscriber, &node);
    rcl_subscription_fini(&payload_command_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

float radps_to_turnsps(float w_radps)
{
    return w_radps * RAD2TURNS; // RAD2TURNS = 1/(2π) ที่ประกาศไว้แล้ว
}

void moveBase()
{
    // อัปเดตเหตุการณ์ CAN (กันค่า feedback ค้าง)
    pumpEvents(can_intf);

    // ถ้าไม่ได้รับคำสั่งนานเกิน timeout ให้หยุดล้อ
    const bool timeout = (millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS;

    // แปลง rad/s -> turns/s + จำกัดเพดานความเร็ว
    float vL_tps = timeout ? 0.0f : radps_to_turnsps(g_cmd_left_radps);
    float vR_tps = timeout ? 0.0f : radps_to_turnsps(g_cmd_right_radps);

    vL_tps = constrain(vL_tps, -VEL_MAX_TURNS, VEL_MAX_TURNS);
    vR_tps = constrain(vR_tps, -VEL_MAX_TURNS, VEL_MAX_TURNS);
    joint_state_msg.effort.data[J_LEFT_WHEEL] = vL_tps;
    joint_state_msg.effort.data[J_RIGHT_WHEEL] = vR_tps;

    // สั่งเฉพาะตอนที่อยู่โหมดที่ล้อเข้า CLOSED_LOOP_CONTROL แล้ว (เช่น SLIDE/OPERATING)
    if (robot_state == SLIDE || robot_state == OPERATING)
    {
        // หมายเหตุ: ฟังก์ชันในไลบรารี ODriveCAN อาจเป็น setInputVel(v) หรือ setInputVel(v, vel_ff)
        // ถ้าของไฟต์เป็น 2 อาร์กิวเมนต์ ให้ส่ง 0.0f เป็น feedforward
        odrv_left_wheel.setVelocity(-vL_tps /*, 0.0f*/);
        odrv_right_wheel.setVelocity(vR_tps /*, 0.0f*/);
    }
    else
    {
        // โหมดอื่นไม่ขยับล้อ
        odrv_left_wheel.setVelocity(0.0f /*, 0.0f*/);
        odrv_right_wheel.setVelocity(0.0f /*, 0.0f*/);
    }
}

void moveArm()
{
}

void publish_joint_state()
{
    // อัปเดต CAN events
    pumpEvents(can_intf);

    // timestamp
    auto ts = getTime();
    joint_state_msg.header.stamp.sec = (int32_t)ts.tv_sec;
    joint_state_msg.header.stamp.nanosec = (uint32_t)ts.tv_nsec;

    // 1) waist (รายงานค่าที่สั่งล่าสุดเป็น position)
    joint_state_msg.position.data[J_LEFT_WAIST] = waist_cmd_deg_left * M_PI / 180.f; // rad
    joint_state_msg.velocity.data[J_LEFT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_LEFT_WAIST] = 0.0;

    joint_state_msg.position.data[J_RIGHT_WAIST] = waist_cmd_deg_right * M_PI / 180.f; // rad
    joint_state_msg.velocity.data[J_RIGHT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_RIGHT_WAIST] = 0.0;

    // 2) ODrive feedback → hip/knee/wheel
    struct Map
    {
        ODriveCAN *drv;
        ODrvData *data;
        int pos_idx;
    } maps[] = {
        {&odrv_left_hip, &D_left_hip, J_LEFT_HIP},
        {&odrv_left_knee, &D_left_knee, J_LEFT_KNEE},
        {&odrv_left_wheel, &D_left_wheel, J_LEFT_WHEEL},
        {&odrv_right_hip, &D_right_hip, J_RIGHT_HIP},
        {&odrv_right_knee, &D_right_knee, J_RIGHT_KNEE},
        {&odrv_right_wheel, &D_right_wheel, J_RIGHT_WHEEL}};

    const float REV2RAD = 2.0f * M_PI;

    for (auto &m : maps)
    {
        Get_Encoder_Estimates_msg_t fb;
        // timeout เล็กๆ 2–5 ms พอ (อย่าใช้ 0 เผื่อเน็ตช้า)
        if (m.drv->getFeedback(fb, 3))
        {
            joint_state_msg.position.data[m.pos_idx] = fb.Pos_Estimate * REV2RAD;
            joint_state_msg.velocity.data[m.pos_idx] = fb.Vel_Estimate * REV2RAD;
            // joint_state_msg.effort.data[m.pos_idx] = 0.0;
        }
        else
        {
            // ถ้าพลาดรอบนี้ ใช้ค่าเดิม (ไม่ทับเป็นศูนย์ เพื่อไม่ให้กราฟกระโดด)
            // ปล่อยว่างก็ได้เพราะเรากำลังรีเฟรชทุก 20 ms อยู่แล้ว
        }
    }

    joint_state_msg.effort.data[J_RIGHT_KNEE] = robot_state;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
}

void publish_imu()
{
    if (!imu_ready)
        return;

    // ถ้า IMU reset ให้ตั้ง report ใหม่
    if (myIMU.wasReset())
    {
        imuSetReports();
    }

    // อ่าน event ใหม่จาก BNO08x
    if (!myIMU.getSensorEvent())
    {
        // ยังไม่มีข้อมูลใหม่ รอบนี้ไม่ publish
        return;
    }

    // เราสนใจเฉพาะ Geomagnetic Rotation Vector
    if (myIMU.getSensorEventID() != SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR)
    {
        return;
    }

    // ได้ roll, pitch, yaw เป็น "เรเดียน" จากไลบรารี SparkFun
    float roll = myIMU.getRoll();   // rad
    float pitch = myIMU.getPitch(); // rad
    float yaw = myIMU.getYaw();     // rad

    // แปลง RPY → Quaternion (ลำดับหมุนแบบ ZYX: yaw → pitch → roll)
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // ตอนนี้ยังไม่ใช้ gyro/acc → ใส่เป็น 0 ไปก่อน
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    // ใส่ timestamp เดียวกับ system (ใช้ฟังก์ชัน getTime() ที่ไฟต์มีอยู่แล้ว)
    auto ts = getTime();
    imu_msg.header.stamp.sec = (int32_t)ts.tv_sec;
    imu_msg.header.stamp.nanosec = (uint32_t)ts.tv_nsec;

    // publish
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void publishData()
{
    publish_joint_state();
    publish_imu();
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

void setServoAngle(uint8_t channel, float angle)
{
    angle = constrain(angle, 0, 180);

    uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(channel, 0, pulselen);
}

void setWaistAnglesDeg(float left_deg, float right_deg)
{
    waist_cmd_deg_left = left_deg;
    waist_cmd_deg_right = right_deg;
    setServoAngle(LEFT_WAIST_CH, waist_cmd_deg_left);
    setServoAngle(RIGHT_WAIST_CH, waist_cmd_deg_right);
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

void wheels_closed_loop_only_safe()
{
    // ขา/สะโพก = IDLE
    odrv_right_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_left_hip.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_right_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odrv_left_knee.setState(ODriveAxisState::AXIS_STATE_IDLE);

    // ล้อ: เข้า CLOSED_LOOP_CONTROL แบบมี retry
    bool ok_r = request_state_with_retry(
        odrv_right_wheel, D_right_wheel,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    bool ok_l = request_state_with_retry(
        odrv_left_wheel, D_left_wheel,
        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // ถ้าด้านไหนไม่โอเค จะ revert เป็น IDLE ป้องกันการค้างครึ่งๆ
    if (!ok_r)
        odrv_right_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
    if (!ok_l)
        odrv_left_wheel.setState(ODriveAxisState::AXIS_STATE_IDLE);
}

void imuSetReports()
{
    if (!imu_ready)
        return;

    // เปิด Geomagnetic Rotation Vector (มีการชดเชยเอียง ให้ yaw เป็น compass ที่ชดเชย roll/pitch แล้ว)
    if (myIMU.enableGeomagneticRotationVector())
    {
        // OK
    }
    else
    {
        // ถ้าเปิดไม่ได้ให้ mark ว่าไม่ ready
        imu_ready = false;
    }
}