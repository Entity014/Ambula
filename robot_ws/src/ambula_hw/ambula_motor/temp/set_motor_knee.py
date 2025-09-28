#!/usr/bin/env python3
import odrive
from odrive.enums import ControlMode, AxisState, EncoderMode
from odrive.utils import dump_errors
import time

def main():
    # 1. ค้นหาและเชื่อมต่อ ODrive
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("❌ ไม่พบ ODrive")
        return

    odrv0.config.brake_resistance         = 2.0   # Ω
    odrv0.config.dc_max_positive_current = 30.0  # A
    odrv0.config.dc_max_negative_current = -2.0  # A
    odrv0.config.max_regen_current       = 0.0   # A
    odrv0.save_configuration()

    # 3. เซ็ตค่ามอเตอร์และคอนโทรลเลอร์ของ axis0
    axis = odrv0.axis0
    # — Controller config
    ctrl = axis.controller.config
    ctrl.pos_gain            = 40.0
    ctrl.vel_gain            = 0.04
    ctrl.vel_integrator_gain = 0.1
    ctrl.control_mode        = ControlMode.POSITION_CONTROL
    ctrl.vel_limit           = 25.0    # rev/s
    # — Motor config
    axis.motor.config.current_lim = 50.0   # A
    axis.motor.config.pole_pairs  = 7
    axis.motor.config.direction   = 1
    axis.motor.config.torque_constant = 8.23 / 120
    axis.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 120)
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    # axis.motor.config.pre_calibrated = True
    # odrv0.save_configuration()
    # ! SENSORLESS
    # axis.requested_state = AxisState.SENSORLESS_CONTROL
    # axis.config.startup_sensorless_control = True

if __name__ == "__main__":
    main()
