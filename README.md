# 2-Leg Wheel Robot Workflow (No Navigation, With Simulation & RL)

## 1) Low-Level Control & Drivers
**Architecture**
```
ROS 2 Control → Teensy (CAN Master) → ODrive (CAN Slave) → BLDC Motors
                          │
                          └── Servo Waist (PWM/Serial)
```
- **Teensy**: Interface ระหว่าง ROS 2 Control และ ODrive, รวมถึงควบคุม Servo
- **ODrive**: รับคำสั่งจาก Teensy ผ่าน CAN bus → คุม BLDC แบบ FOC

**Joint & Control Mode**
| Joint   | Motor Type | Control Mode                    | เหตุผล |
|---------|-----------|----------------------------------|--------|
| Waist   | Servo     | Position Control (Absolute)      | หมุนเอว, ไม่ต้องการแรงสูง |
| Hip     | BLDC      | Position Control (Cascaded)      | Outer pos loop → inner vel/current loop |
| Knee    | BLDC      | Position Control (Cascaded)      | เหมือน Hip |
| Wheel   | BLDC      | Torque Control (FOC)             | สำหรับ balancing & drive |

**ROS 2 Control Mapping**
```yaml
joints:
  - waist_joint: position
  - hip_joint:   position
  - knee_joint:  position
  - wheel_joint: effort
```

---

## 2) State Estimation
**Input Sensor**
- IMU: D435i IMU + BNO055
- Wheel Encoders
- (Optional) VIO: VINS-Mono/VINS-Fusion

**Processing**
- Mahony / nonlinear complementary filter → roll, pitch, yaw, yaw_rate
- Encoder + IMU → linear velocity
- VINS → ลด drift ของ odometry
- Contact detection → จาก encoder/current/IMU shock detection

**Output (State Vector ตัวอย่าง)**
```
[pitch, pitch_rate, linear_velocity, yaw_rate, height, contact_state]
```

---

## 3) Balancing & Motion Control
**Controller**
- LQR สำหรับ balance & drive
- MPPI (information-theoretic MPC) สำหรับ nonlinear dynamics

**State Machine หลัก**
```
balance
 ├─ drive_forward / drive_backward
 ├─ jump (crouch → thrust → flight → land)
 ├─ step_up (approach → lift → place → push up → land)
 ├─ step_down (approach → descend → land)
 ├─ height_adjust (low/mid/high stance)
 └─ fall_recover (detect fall → posture adjust → stand up)
```

---

## 4) Perception (Obstacle / Stair Detection)
**Goal:** ตรวจจับบันได/อุปสรรคเพื่อปรับความสูงหรือเปลี่ยนโหมดการเคลื่อนที่

**Methods**
1. Geometric: Depth/LiDAR → RANSAC ground removal → normal clustering → riser/tread
2. Deep Learning: StairNet-RGBD → segmentation tread/riser → step parameters

**Output**
```
StepInfo: {riser_mean, tread_mean, n_steps, confidence}
```

---

## 5) Safety & Operations
- Wireless E-Stop → `/safety/stop`
- Watchdog → monitor pitch/yaw_rate, torque, encoder fault
- Emergency stop interrupt ทุก state
- Logger → เก็บข้อมูลเพื่อตรวจสอบและปรับ controller

---

## 6) Simulation & RL
**Simulator**
- MuJoCo / IsaacLab

**Training**
- PPO baseline: balance, drive, jump, step_up, fall_recover
- RMA: ปรับตัวในโลกจริง
- Domain Randomization: mass, friction, latency, sensor noise

**Integration**
- Export RL policy เป็น ROS 2 node
- ผนวก policy เข้ากับ Balancer/State Machine

---

## 7) ROS 2 Workspace Structure
```
ambula_ws/src/
  ambula_hw/                # Teensy/ODrive driver
  ambula_balancer/          # State machine + LQR + MPPI + RL integration
  ambula_state_estimator/   # Mahony + VINS wrapper
  ambula_perception_step/   # Stair/obstacle detection
  ambula_safety/            # E-Stop, watchdog
  ambula_rl/                # Training + inference scripts
  ambula_bringup/           # Launch + params
```

---

## 8) Data Flow
```
IMU + Encoders ──> State Estimator ──┐
Depth/LiDAR  ──> Perception          │
                                      ├─> Balancer / State Machine (balance/drive/jump/step_up/step_down/height_adjust/fall_recover)
Simulation + RL <────────────────────┘
                                      ├─> ROS 2 Control ──> Teensy ──> ODrive ──> Motors/Servo
Safety (E-Stop, Watchdog) ────────────┘
```
