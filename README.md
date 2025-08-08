# 2-Leg Wheel Robot Workflow (With Simulation & RL)

## 1) Low-Level Control & Drivers

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
| Joint   | Motor Type     | Control Mode               | Feedback Sensor |
|---------|---------------|----------------------------|-----------------|
| Waist   | Servo 360°     | Position Control (Absolute)| AS5048A         |
| Hip     | BLDC (ODrive)  | Position Control (Cascaded)| AS5048A         |
| Knee    | BLDC (ODrive)  | Position Control (Cascaded)| AS5048A         |
| Wheel   | BLDC (ODrive)  | Torque Control (FOC)       | AS5048A         |

**Reason for AS5048A on all joints**
- Absolute encoder → no homing needed, no drift
- High resolution (~0.0219°/step)
- SPI interface → low latency

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
- Absolute encoders (AS5048A) on all joints
- (Optional) VIO: VINS-Mono/VINS-Fusion

**Processing**
- Mahony / nonlinear complementary filter → roll, pitch, yaw, yaw_rate
- Encoder + IMU → linear velocity
- VINS → reduce odometry drift
- Contact detection via encoder/current/IMU shock

**Output (State Vector Example)**
```
[pitch, pitch_rate, linear_velocity, yaw_rate, height, contact_state]
```

---

## 3) Balancing & Motion Control

**Controller**
- LQR: balance & drive
- MPPI: nonlinear dynamics handling

**Main State Machine**
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

**Goal:** Detect stairs/obstacles → adjust height or change motion mode

**Methods**
1. **Geometric:** Depth/LiDAR → voxel downsample → ROI crop → RANSAC ground removal → normal clustering → riser/tread
2. **Deep Learning:** StairNet-RGBD → tread/riser segmentation → post-process step parameters

**Output**
```
StepInfo: {riser_mean, tread_mean, n_steps, confidence}
```

---

## 5) Safety & Operations
- Wireless E-Stop → `/safety/stop`
- Watchdog: monitor pitch/yaw_rate, torque, encoder fault, contact loss
- Emergency stop interrupts any state
- Logger: store IMU, odometry, torque for system ID & tuning

---

## 6) Simulation & RL

**Simulator**
- MuJoCo / IsaacLab

**Training**
- PPO baseline: balance, drive, jump, step_up, fall_recover
- Rapid Motor Adaptation (RMA): real-world adaptation
- Domain Randomization: mass, friction, latency, sensor noise

**Integration**
- Export RL policy as ROS 2 node
- Integrate with Balancer/State Machine (hybrid control)

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
