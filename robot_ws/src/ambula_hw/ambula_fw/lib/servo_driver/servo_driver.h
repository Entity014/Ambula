#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Servo_Driver
{
public:
    Servo_Driver(float min_pulses, float max_pulses, float freq);
    bool init();
    void setServoAngle(uint8_t servo_num, float angle);

private:
    bool startActuator();
    float min_pulses_;
    float max_pulses_;
    float freq_;
};

#endif