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
    ret = servo_actuator.begin();
    servo_actuator.setPWMFreq(freq_);
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

void Servo_Driver::setServoAngle(uint8_t servo_num, float angle)
{
    servo_actuator.setPWM(servo_num, 0, map(angle, 0, 180, min_pulses_, max_pulses_));
}