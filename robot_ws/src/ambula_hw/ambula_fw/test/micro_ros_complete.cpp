#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "config.h"
#include "imu.h"
#include "odometry.h"
#include "kinematics.h"
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
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>

// ---------- ODrive CAN ----------
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // teensy compile hack

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
rcl_publisher_t odom_publisher;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t debug_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t leg_subscriber;
rcl_subscription_t robot_command_subscriber;
rcl_subscription_t led_command_subscriber;
rcl_subscription_t payload_command_subscriber;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Int8 robot_command_msg;
std_msgs__msg__Int8 led_command_msg;
std_msgs__msg__Bool payload_command_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist leg_msg;
geometry_msgs__msg__Twist debug_msg;

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

Kinematics kinematics(
    Kinematics::DIFFERENTIAL_DRIVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    RPM_RATIO,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

Odometry odometry;
IMU imuSensor;
Servo_Driver servoActuator(SERVOMIN, SERVOMAX, SERVO_FREQ);

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

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void moveBase();
void moveActuator();
void publishData();
bool createEntities();
bool destroyEntities();

struct timespec getTime();

//------------------------------ < Rapid Define > -------------------------------------//

float waist_arr[2] = {90.f, 90.f}; // left, right

void setup()
{
    pinMode(LED_PIN, OUTPUT);

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
    servoActuator.setServoAngle(LEFT_WAIST_CH, waist_arr[0]);  // left waist
    servoActuator.setServoAngle(RIGHT_WAIST_CH, waist_arr[1]); // right waist

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

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0)
    {
        prev_cmd_time = millis();
    }
}

void legCallback(const void *msgin)
{
}

void robotCommandCallback(const void *msgin)
{
    auto *m = (const std_msgs__msg__Int8 *)msgin;
    switch (m->data)
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

void ledCommandCallback(const void *msgin)
{
}

void payloadCommandCallback(const void *msgin)
{
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        moveBase();
        moveActuator();
        publishData();
    }
}

void moveBase()
{
    if (((millis() - prev_cmd_time) >= 500))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        digitalWrite(LED_PIN, HIGH);
    }

    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);

    // TODO: รับค่า velocity
    debug_msg.linear.x = req_rpm.motor1 / 60.f;
    debug_msg.linear.y = req_rpm.motor2 / 60.f;
    debug_msg.linear.z = req_rpm.motor3 / 60.f;
    debug_msg.angular.x = req_rpm.motor4 / 60.f;

    // TODO: สั่งความเร็วล้อ
    // motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    // motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    // motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    // motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    // Kinematics::velocities current_vel = kinematics.getVelocities(
    //     current_rpm1,
    //     current_rpm2,
    //     current_rpm3,
    //     current_rpm4);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    // odometry.update(
    //     vel_dt,
    //     current_vel.linear_x,
    //     current_vel.linear_y,
    //     current_vel.angular_z);
}

void moveActuator()
{
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imuSensor.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    joint_state_msg.position.data[J_LEFT_WAIST] = waist_arr[0] * DEG_TO_RAD; // rad
    joint_state_msg.velocity.data[J_LEFT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_LEFT_WAIST] = 0.0;

    joint_state_msg.position.data[J_RIGHT_WAIST] = waist_arr[1] * DEG_TO_RAD; // rad
    joint_state_msg.velocity.data[J_RIGHT_WAIST] = 0.0;
    joint_state_msg.effort.data[J_RIGHT_WAIST] = 0.0;

    pumpEvents(can_intf);

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

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
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
        &debug_publisher,
        &node,
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

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(
        &leg_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_joint"));
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
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &leg_subscriber,
        &leg_msg,
        &legCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &robot_command_subscriber,
        &robot_command_msg,
        &robotCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &led_command_subscriber,
        &led_command_msg,
        &ledCommandCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &payload_command_subscriber,
        &payload_command_msg,
        &payloadCommandCallback,
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

    servoActuator.setServoAngle(LEFT_WAIST_CH, 90.0f);  // left waist
    servoActuator.setServoAngle(RIGHT_WAIST_CH, 90.0f); // right waist

    rosidl_runtime_c__String__fini(&joint_state_msg.header.frame_id);
    rosidl_runtime_c__String__Sequence__fini(&joint_state_msg.name);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.position);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.velocity);
    rosidl_runtime_c__double__Sequence__fini(&joint_state_msg.effort);

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_publisher_fini(&debug_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&leg_subscriber, &node);
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