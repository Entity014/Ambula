
#include <Arduino.h>
#include "ODriveCAN.h"

#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0

#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

void onCanMessage(const CanMsg &msg);

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

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN *odrives[] = {&odrv0};                         // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData
{
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
};

ODriveUserData odrv0_user_data;

void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data)
{
    ODriveUserData *odrv_user_data = static_cast<ODriveUserData *>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg &msg)
{
    for (auto odrive : odrives)
    {
        onReceive(msg, *odrive);
    }
}

void setup()
{
    Serial.begin(115200);

    // Wait for up to 3 seconds for the serial port to be opened on the PC side.
    // If no PC connects, continue anyway.
    for (int i = 0; i < 30 && !Serial; ++i)
    {
        delay(100);
    }
    delay(200);

    Serial.println("Starting ODriveCAN demo");

    // Register callbacks for the heartbeat and encoder feedback messages
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCan())
    {
        Serial.println("CAN failed to initialize: reset required");
        while (true)
            ; // spin indefinitely
    }

    Serial.println("Waiting for ODrive...");
    while (!odrv0_user_data.received_heartbeat)
    {
        pumpEvents(can_intf);
    }

    Serial.println("found ODrive");

    // request bus voltage and current (1sec timeout)
    Serial.println("attempting to read bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus;
    if (!odrv0.request(vbus, 1000))
    {
        Serial.println("vbus request failed!");
        while (true)
            ; // spin indefinitely
    }

    Serial.print("DC voltage [V]: ");
    Serial.println(vbus.Bus_Voltage);
    Serial.print("DC current [A]: ");
    Serial.println(vbus.Bus_Current);

    Serial.println("Enabling closed loop control...");
    // while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
    // {
    //     odrv0.clearErrors();
    //     delay(1);
    //     odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    //     // Pump events for 150ms. This delay is needed for two reasons;
    //     // 1. If there is an error condition, such as missing DC power, the ODrive might
    //     //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //     //    on the first heartbeat response, so we want to receive at least two
    //     //    heartbeats (100ms default interval).
    //     // 2. If the bus is congested, the setState command won't get through
    //     //    immediately but can be delayed.
    //     for (int i = 0; i < 15; ++i)
    //     {
    //         delay(10);
    //         pumpEvents(can_intf);
    //     }
    // }

    Serial.println("ODrive running!");
}

void loop()
{
    // (ถ้าอยากให้ปลอดภัย ใส่ pumpEvents ไว้ด้วย แม้ Teensy จะใช้ interrupt ก็ตาม)
    pumpEvents(can_intf);

    // odrv0.setTorque(0.1f); // ปล่อยให้มอเตอร์ฟรี

    // วิธีที่ 2: poll ขอค่าเอง (ใช้ได้ทันทีแม้ยังไม่เปิด feedback messages)
    Get_Encoder_Estimates_msg_t fb2;
    if (odrv0.getFeedback(fb2, 50))
    { // timeout 50 ms
        Serial.print("poll-pos:");
        Serial.print(fb2.Pos_Estimate);
        Serial.print(",poll-vel:");
        Serial.println(fb2.Vel_Estimate);
    }
}
