#ifndef ROBOT_H
#define ROBOT_H

#define USE_BNO086_IMU
#define DIFFERENTIAL_DRIVE_ROBOT

#define ROS_DOMAIN_ID 15
#define CAN_BAUDRATE 250000

#define MOTOR_MAX_RPM 800.0f
#define MAX_RPM_RATIO 0.8f
#define RPM_RATIO 1.0f
#define WHEEL_DIAMETER 0.15f     // meter
#define LR_WHEELS_DISTANCE 0.45f // meter

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz
#define LEFT_WAIST_CH 9
#define RIGHT_WAIST_CH 8
#define LED_CH 4

#define LED_PIN 13
#define ESTOP_PIN 9
#define PAYLOAD_PIN 26

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

#define ODRV_RIGHT_HIP_ID 0   // Right Hip
#define ODRV_LEFT_HIP_ID 2    // Left Hip
#define ODRV_RIGHT_KNEE_ID 3  // Right Knee
#define ODRV_LEFT_KNEE_ID 5   // Left Knee
#define ODRV_RIGHT_WHEEL_ID 4 // Right Wheel
#define ODRV_LEFT_WHEEL_ID 6  // Left Wheel

#endif