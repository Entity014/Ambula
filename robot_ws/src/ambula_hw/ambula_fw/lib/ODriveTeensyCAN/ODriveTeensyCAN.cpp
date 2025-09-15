#include "ODriveTeensyCAN.h"

// ----- static members -----
ODriveTeensyCAN *ODriveTeensyCAN::instances_[4] = {nullptr, nullptr, nullptr, nullptr};
uint8_t ODriveTeensyCAN::instance_count_ = 0;

// ----- ctor -----
ODriveTeensyCAN::ODriveTeensyCAN(TeensyCan &can, uint32_t node_id, uint32_t baud)
    : can_(can),
      baud_(baud),
      node_id_(node_id),
      odrv_(wrap_can_intf(can), node_id)
{
    // เก็บ instance สำหรับ multiplexer RX
    if (instance_count_ < 4)
    {
        instances_[instance_count_++] = this;
    }
}

// ----- begin -----
bool ODriveTeensyCAN::begin()
{
    can_.begin();
    can_.setBaudRate(baud_);
    can_.setMaxMB(16);
    can_.enableFIFO();
    can_.enableFIFOInterrupt();
    can_.onReceive(ODriveTeensyCAN::onCanRxISR);

    // ติดตั้ง callback ของ ODrive
    odrv_.onStatus(ODriveTeensyCAN::onHeartbeatISR, this);
    odrv_.onFeedback(ODriveTeensyCAN::onFeedbackISR, this);
    return true;
}

// ----- polling helpers -----
void ODriveTeensyCAN::onCanRxISR(const CanMsg &msg)
{
    // ส่งต่อไปยังทุก instance ที่ลงทะเบียนไว้
    for (uint8_t i = 0; i < instance_count_; ++i)
    {
        if (instances_[i])
        {
            onReceive(msg, instances_[i]->odrv_);
        }
    }
}

void ODriveTeensyCAN::onHeartbeatISR(Heartbeat_msg_t &msg, void *ctx)
{
    auto *self = static_cast<ODriveTeensyCAN *>(ctx);
    self->hb_.raw = msg;
    self->hb_.last_ms = millis();
    self->hb_.seen = true;
}

void ODriveTeensyCAN::onFeedbackISR(Get_Encoder_Estimates_msg_t &msg, void *ctx)
{
    auto *self = static_cast<ODriveTeensyCAN *>(ctx);
    self->fb_.pos = msg.Pos_Estimate;
    self->fb_.vel = msg.Vel_Estimate;
    self->fb_.last_ms = millis();
    self->fb_.fresh = true;
}

// ----- high-level ops -----
bool ODriveTeensyCAN::enableClosedLoop(uint32_t timeout_ms)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms)
    {
        odrv_.clearErrors();
        odrv_.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        // รอ heartbeat อย่างน้อย 150 ms (ค่าเริ่มต้น heartbeat 100 ms)
        for (int i = 0; i < 15; ++i)
        {
            delay(10);
            pumpEvents(can_);
        }

        if (hb_.seen &&
            hb_.raw.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            return true;
        }
    }
    return false;
}

bool ODriveTeensyCAN::readBusVbus(float &vbus, float &ibus, uint32_t timeout_ms)
{
    Get_Bus_Voltage_Current_msg_t v;
    if (!odrv_.request(v, timeout_ms))
        return false;
    vbus = v.Bus_Voltage;
    ibus = v.Bus_Current;
    return true;
}

bool ODriveTeensyCAN::requestEncoder(float &pos, float &vel, uint32_t timeout_ms)
{
    Get_Encoder_Estimates_msg_t fb;
    if (!odrv_.getFeedback(fb, timeout_ms))
        return false;
    pos = fb.Pos_Estimate;
    vel = fb.Vel_Estimate;
    return true;
}

void ODriveTeensyCAN::setTorque(float torque)
{
    odrv_.setTorque(torque);
}
