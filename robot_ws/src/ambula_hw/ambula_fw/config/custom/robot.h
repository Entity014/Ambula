#ifndef ROBOT_H
#define ROBOT_H

#define USE_BNO086_IMU
#define DIFFERENTIAL_DRIVE_ROBOT

#define ROS_DOMAIN_ID 15

#define MOTOR_MAX_RPM 800.0f
#define MAX_RPM_RATIO 0.8f
#define RPM_RATIO 1.0f
#define WHEEL_DIAMETER 0.15f     // meter
#define LR_WHEELS_DISTANCE 0.45f // meter

#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz

#define LED_PIN 13

#endif