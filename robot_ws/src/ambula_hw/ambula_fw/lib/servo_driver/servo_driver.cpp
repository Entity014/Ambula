#include "servo_driver.h"

Servo_Driver::Servo_Driver(float min_pulses, float max_pulses, float freq)
    : min_pulses_(min_pulses),
      max_pulses_(max_pulses),
      freq_(freq)
{
}

bool Servo_Driver::startActuator()
{
    Wire.begin();

    bool ret;
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    ret = pwm.begin();
    pwm.setPWMFreq(freq_);
    if (!ret)
    {
        return false;
    }

    return true;
}

bool Servo_Driver::init()
{
    bool actuator_ok = startActuator();
    return actuator_ok;
}

void Servo_Driver::setAngle(uint8_t servo_num, float angle)
{
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

    float pulse_length = min_pulses_ + (max_pulses_ - min_pulses_) * (angle / 180.0f);
    uint16_t pulse = static_cast<uint16_t>((pulse_length / 1000000.0f) * freq_ * 4096.0f);

    pwm.setPWM(servo_num, 0, pulse);
}