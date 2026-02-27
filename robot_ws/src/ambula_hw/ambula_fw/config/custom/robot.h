#ifndef ROBOT_H
#define ROBOT_H

#define USE_BNO086_IMU
#define DIFFERENTIAL_DRIVE_ROBOT

#define ROS_DOMAIN_ID 15
#define CAN_BAUDRATE 250000

#define MOTOR_MAX_RPM 800.0f
#define MAX_RPM_RATIO 0.8f
#define RPM_RATIO 1.0f
#define WHEEL_DIAMETER 0.14f     // meter
#define LR_WHEELS_DISTANCE 0.27f // meter

#define LEG_L1 0.2f
#define LEG_L2 0.24f
#define LEG_L3 0.0f
#define LEG_HIP_X 0.0f
#define LEG_HIP_Y_LEFT 0.2f
#define LEG_HIP_Y_RIGHT 0.2f
#define LEG_HIP_Z 0.0f
#define LEG_ALIGN_Y (float)M_PI * 0.5f
#define WAIST_OFFSET_LEFT 0.0f
#define HIP_OFFSET_LEFT 0.0f
#define KNEE_OFFSET_LEFT 0.0f
#define WAIST_OFFSET_RIGHT 0.0f
#define HIP_OFFSET_RIGHT 0.0f
#define KNEE_OFFSET_RIGHT 0.0f

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz
#define LEFT_WAIST_CH 9
#define RIGHT_WAIST_CH 8
#define LED_CH 4

#define LED_PIN 13
#define INTERRUPT_PIN 40
#define RESET_PIN 33
#define ESTOP_PIN 9
#define PAYLOAD_PIN 26
#define PAYLOAD_PULSE_MS 120

#define N_JOINTS 8
const char *JOINT_NAMES[N_JOINTS] = {
    "left_waist_joint", "left_hip_joint", "left_knee_joint", "left_wheel_joint",
    "right_waist_joint", "right_hip_joint", "right_knee_joint", "right_wheel_joint"};

#define ODRV_RIGHT_HIP_ID 0   // Right Hip [0]
#define ODRV_LEFT_HIP_ID 2    // Left Hip [2]
#define ODRV_RIGHT_KNEE_ID 3  // Right Knee [3]
#define ODRV_LEFT_KNEE_ID 5   // Left Knee [5]
#define ODRV_RIGHT_WHEEL_ID 4 // Right Wheel [1]
#define ODRV_LEFT_WHEEL_ID 6  // Left Wheel [4]

#define MOTOR_TORQUE_CONSTANT 0.06858f
#define ENCODER_CPR 16384.0f

#define POS_GAIN_RIGHT_HIP 0.01f
#define VEL_GAIN_RIGHT_HIP 0.0f
#define VEL_INTEGRAL_GAIN_RIGHT_HIP 0.0f

#define POS_GAIN_LEFT_HIP 0.0f
#define VEL_GAIN_LEFT_HIP 0.0f
#define VEL_INTEGRAL_GAIN_LEFT_HIP 0.0f

#define POS_GAIN_RIGHT_KNEE 5.0f
#define VEL_GAIN_RIGHT_KNEE 0.005f
#define VEL_INTEGRAL_GAIN_RIGHT_KNEE 0.0f

#define POS_GAIN_LEFT_KNEE 5.0f
#define VEL_GAIN_LEFT_KNEE 0.005f
#define VEL_INTEGRAL_GAIN_LEFT_KNEE 0.0f

#define TAU_MAX 1.0f

#endif