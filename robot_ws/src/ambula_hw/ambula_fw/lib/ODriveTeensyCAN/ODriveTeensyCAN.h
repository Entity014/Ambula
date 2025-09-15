#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"

// เลือก CAN1/CAN2 ตามบอร์ดที่ใช้
using TeensyCan = FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;

class ODriveTeensyCAN
{
public:
    struct Feedback
    {
        float pos = 0.f;
        float vel = 0.f;
        uint32_t last_ms = 0;
        bool fresh = false;
    };

    struct Heartbeat
    {
        Heartbeat_msg_t raw{};
        uint32_t last_ms = 0;
        bool seen = false;
    };

    ODriveTeensyCAN(TeensyCan &can, uint32_t node_id, uint32_t baud = 250000);

    // เริ่มระบบ CAN + ลงทะเบียน callback
    bool begin();

    // ต้องเรียกใน loop เสมอ (แม้ใช้ interrupt ก็ช่วยเคลียร์คิว)
    inline void poll() { pumpEvents(can_); }

    // เข้าสู่ CLOSED_LOOP_CONTROL (จะ retry ภายในช่วงเวลาที่กำหนด)
    bool enableClosedLoop(uint32_t timeout_ms = 1000);

    // ขอค่า bus voltage/current แบบ synchronous (มี timeout)
    bool readBusVbus(float &vbus, float &ibus, uint32_t timeout_ms = 200);

    // ขอ encoder pos/vel แบบ synchronous (มี timeout)
    bool requestEncoder(float &pos, float &vel, uint32_t timeout_ms = 50);

    // ตั้งทอร์กชั่วคราว (เดโม่)
    void setTorque(float torque);

    // ข้อมูลสถานะล่าสุด (จาก callback)
    const Feedback &feedback() const { return fb_; }
    const Heartbeat &heartbeat() const { return hb_; }

private:
    static void onHeartbeatISR(Heartbeat_msg_t &msg, void *ctx);
    static void onFeedbackISR(Get_Encoder_Estimates_msg_t &msg, void *ctx);
    static void onCanRxISR(const CanMsg &msg); // เดินต่อให้หลาย instance ได้

    TeensyCan &can_;
    uint32_t baud_;
    uint32_t node_id_;

    ODriveCAN odrv_;
    Feedback fb_;
    Heartbeat hb_;

    // สำหรับ static onCanRxISR
    static ODriveTeensyCAN *instances_[4];
    static uint8_t instance_count_;
};
