# 2-Leg Wheel Robot Workflow (With Simulation, Networking & QoS)

## 1) Networking & Communication

**Topology**

```
           ┌─────────────── Laptop / Dev PC (Wi-Fi)
           │                     ↑
           │                 (ROS 2 peer)
           │                     │
        ┌──┴───────────── Wi-Fi AP/Router
        │
   ┌────┴───────┐     ┌─────────────────────────┐
   │  Switch    │────▶│  Pi-A (Perception/VIO)  │── D435i
   │ (GigE)     │◀────│   + Wi-Fi uplink to PC  │
   └────┬───────┘     └─────────────────────────┘
        │
        └────────────── Pi-B (Control/Estimator/Safety) ── Teensy (Micro ROS) ── ODrive/Servo (CAN BUS)
                                         │
                                         └─ SPI ── AS5048A (ทุก joint)
```

**Roles**

- **Pi-A:** ทำ perception pipeline + (optional) VINS-Fusion, ส่ง StepInfo/Odom → Pi-B
- **Pi-B:** ทำ state estimation, balancing, motion control, safety, ROS 2 control bridge → Teensy/ODrive
- **PC:** monitoring, teleop, logging (รับข้อมูลที่จำเป็นผ่าน Wi-Fi)

**Time Sync**

- ใช้ `chrony` sync เวลา
- ถ้าต้องการ jitter < 1 ms → ใช้ PTP (IEEE-1588)

---

## 2) Low-Level Control & Drivers

**Architecture**

```
ROS 2 Control
   │
   └── Teensy (CAN Master + SPI Master)
         │
         ├── Waist: Servo 360° ← PWM
         │        └── AS5048A (SPI)
         │
         ├── Hip: BLDC ← CAN → ODrive → FOC
         │      └── AS5048A (SPI)
         │
         ├── Knee: BLDC ← CAN → ODrive → FOC
         │       └── AS5048A (SPI)
         │
         └── Wheel: BLDC ← CAN → ODrive → FOC
                 └── AS5048A (SPI)
```

**Joint & Control Mode**

| Joint | Motor Type | Control Mode                | Feedback Sensor |
| ----- | ---------- | --------------------------- | --------------- |
| Waist | Servo 360° | Position Control (Absolute) | AS5048A         |
| Hip   | BLDC       | Position Control (Cascaded) | AS5048A         |
| Knee  | BLDC       | Position Control (Cascaded) | AS5048A         |
| Wheel | BLDC       | Torque Control (FOC)        | AS5048A         |

---

## 3) State Estimation

**Input Sensor**

- IMU: D435i + BNO055
- Absolute encoders (AS5048A) ทุก joint
- (Optional) VIO: VINS-Mono/VINS-Fusion

**Processing**

- Mahony / nonlinear complementary filter → roll, pitch, yaw, yaw_rate
- Encoder + IMU → linear velocity
- VINS → ลด drift ของ odometry
- **ไม่มี Contact detection ในหุ่นยนต์จริง**

**Output (State Vector Example)**

```
[pitch, pitch_rate, linear_velocity, yaw_rate, height]
```

---

## 4) Balancing & Motion Control

**Controller Architecture**

```
               RGB-D Camera
                     │
                     ▼
    ┌─────────────────────────────┐
    │   Perception (Obstacle /    │
    │   Stair Detection)          │
    │  • Geometric / DL detection │
    │  • Output: StepInfo         │
    └─────────────────────────────┘
                     │ StepInfo (riser, tread, n_steps, conf.)
                     ▼
 Encoders           IMU
    │                │
    └───────┐   ┌────┘
            ▼   ▼
     ┌──────────────────┐
     │ State Estimation │
     │ • Fuse encoders  │
     │   & IMU          │
     │ • Pose, vel, ht  │
     └──────────────────┘
              ▲
              │
        User Input / Mission / StepInfo
              │
              ▼
     ┌──────────────────┐
     │ Position         │
     │ Controller       │
     └──────────────────┘
              │
     ┌────────┴─────────┬──────────────┬──────────────────┬───────────────────┐
     ▼                  ▼              ▼                  ▼                   ▼
┌─────────────┐ ┌────────────┐ ┌──────────────┐ ┌───────────────────┐ ┌─────────────────┐
│ Stabilizing │ │ Jump       │ │ Step Up/Down │ │ Height Adjust     │ │ Fall Recovery   │
│ Controller  │ │ Controller │ │ Controller   │ │ Controller        │ │ Controller      │
└────┬────────┘ └───────┬────┘ └───────┬──────┘ └─────────┬─────────┘ └───────┬─────────┘
     │                  │              │                  │                   │
     ▼                  ▼              ▼                  ▼                   ▼
 Wheel Motors       Hip Motors     Wheel + Leg        Leg (Hip/Knee)      Wheel + Leg
```

**Main State Machine**

```
balance
 ├─ drive_forward / drive_backward [Wheel]
 ├─ jump (crouch → thrust → flight → land) [Wheel + Leg]
 ├─ step_up (approach → lift → place → push up → land) [Wheel + Leg]
 ├─ step_down (approach → descend → land) [Wheel + Leg]
 ├─ height_adjust (low/mid/high stance) [Leg]
 └─ fall_recover (detect fall → posture adjust → stand up) [Wheel + Leg]
```

**การทำงานหลัก**

- **Perception:** ตรวจจับบันไดหรือสิ่งกีดขวางเพื่อเลือกโหมด Step Up/Down หรือ Height Adjust
- **State Estimation:** รวม IMU + encoders + optional VIO เพื่อให้ได้ state vector
- **Controllers:** Stabilizing, Jump, Step Up/Down, Height Adjust, Fall Recovery
- **Actuators:** ส่งคำสั่งไปยัง Wheel, Hip/Knee, Waist

---

## 5) Perception (Obstacle / Stair Detection)

**Goal:** ตรวจจับบันไดหรือสิ่งกีดขวาง → ปรับความสูงหรือเปลี่ยนโหมดการเคลื่อนที่

**Methods**

1. **Geometric:** Depth/LiDAR → voxel downsample → ROI crop → RANSAC ground removal → normal clustering → riser/tread
2. **Deep Learning:** StairNet-RGBD → tread/riser segmentation → post-process step parameters

**Output**

```
StepInfo: {riser_mean, tread_mean, n_steps, confidence}
```

---

## 6) Safety & Operations

- Wireless E-Stop → `/safety/stop` (Emergency stop interrupt ทุก state)
- Watchdog: monitor pitch/yaw_rate, torque, encoder fault, contact loss
- (Optional) Logger: เก็บข้อมูล IMU, odometry, torque

---

## 7) Simulation & RL

**Simulator**

- **IsaacLab** → สำหรับ RL training, domain randomization, locomotion control
- **Gazebo** → สำหรับ ROS 2 integration, hardware-in-the-loop, และ scenario simulation

**Training**

- PPO baseline: balance, drive, jump, step_up, fall_recover
- Rapid Motor Adaptation (RMA): real-world adaptation
- Domain Randomization: mass, friction, latency, sensor noise

**Integration**

- Export RL policy เป็น ROS 2 node
- เชื่อมกับ Balancer/State Machine
- Workflow IsaacLab → Gazebo เพื่อลด sim-to-real gap

---

## 8) ROS 2 Workspace Structure

```
ambula_ws/
  src/
    ambula_bringup/            # launch + params + compose scenarios
    ambula_controller/         # LQR/MPPI/RL + state machine
    ambula_hw/                 # hardware interface + bridges
      ambula_fw/               # PlatformIO firmware สำหรับ Teensy
    ambula_perception/         # stair/obstacle detection (RGB‑D/LiDAR)
    ambula_rl/                 # RL training/inference + tools
    ambula_safety/             # E‑Stop, watchdog, fault monitor
    ambula_simulation/
      ambula_description/      # URDF/Xacro + meshes
      ambula_world/            # Gazebo worlds/models
    ambula_state_estimator/    # Mahony + VIO/VINS wrapper
```

---

## 9) Data Flow

```
IMU + Encoders ──> State Estimator ──┐
Depth/LiDAR  ──> Perception          │
                                     ├─> Balancer / State Machine
Simulation + RL <────────────────────┘
                                     ├─> ROS 2 Control ──> Teensy ──> ODrive ──> Motors/Servo
Safety (E-Stop, Watchdog) ───────────┘
```

---

## 10) ROS 2/DDS: QoS & Traffic Profile

**QoS Recommendations**

- **Sensor topics:** `BEST_EFFORT`, `KEEP_LAST`, depth=5–10
- **Control commands:** `RELIABLE`, `KEEP_LAST`, depth=10
- **State/Status:** `RELIABLE`, depth=5

**Traffic Optimization**

- ใช้ `image_transport` แบบ compressed สำหรับภาพบน Wi-Fi
- จำกัด heavy topics ผ่าน Wi-Fi, เก็บดิบใน Pi
- ใช้ Cyclone DDS หรือ Fast DDS, multicast discovery ใน LAN
