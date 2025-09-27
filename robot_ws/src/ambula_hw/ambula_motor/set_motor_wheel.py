#!/usr/bin/env python3
import odrive
from odrive.enums import ControlMode, AxisState, EncoderMode
from odrive.utils import dump_errors
import time

def main():
    print("🔍 กำลังค้นหา ODrive...")
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("❌ ไม่พบ ODrive")
        return
    print("✅ พบ ODrive:", odrv0.serial_number)

    # ==== Board Config ====
    odrv0.config.brake_resistance         = 2.0   # Ω
    odrv0.config.dc_max_positive_current = 30.0  # A
    odrv0.config.dc_max_negative_current = -2.0  # A
    odrv0.config.max_regen_current       = 0.0   # A

    axis = odrv0.axis0

    # ==== Motor Config ====
    axis.motor.config.current_lim = 50.0   # A
    axis.motor.config.pole_pairs  = 7
    axis.motor.config.direction   = 1
    axis.motor.config.torque_constant = 8.23 / 120  # (Nm/A)
    axis.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 120)

    # ==== Controller Config ====
    ctrl = axis.controller.config
    ctrl.pos_gain            = 20.0
    ctrl.vel_gain            = 0.05
    ctrl.vel_integrator_gain = 0.03
    ctrl.control_mode        = ControlMode.POSITION_CONTROL
    ctrl.vel_limit           = 14.0    # rev/s

    # ==== Encoder Config (HALL SENSOR) ====
    axis.encoder.config.mode = EncoderMode.HALL
    axis.encoder.config.cpr = 6 * 7  # 6 hall sensor * pole_pairs
    axis.encoder.config.calib_range = 0.05
    axis.encoder.config.use_index = False

    # ==== Save Configuration ก่อนเริ่ม Calib ====
    print("💾 บันทึก config ก่อนเริ่ม Calibration...")
    odrv0.save_configuration()
    time.sleep(2)

    # ==== Calibration (บังคับทำครั้งแรก) ====
    print("⚙️ กำลัง Calibrate Motor...")
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    while axis.current_state != AxisState.IDLE:
        time.sleep(0.1)

    # ==== Mark ว่าคาลิเบรตแล้ว และเซฟไว้ ====
    axis.motor.config.pre_calibrated = True
    odrv0.save_configuration()
    print("✅ Calibration เสร็จ และบันทึกค่าแล้ว")

    # ==== Error Check ====
    dump_errors(odrv0)

if __name__ == "__main__":
    main()
