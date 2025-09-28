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

    axis = odrv0.axis0
    # — Sensor config
    axis.encoder.config.mode = EncoderMode.HALL
    axis.encoder.config.cpr = 6 * 7  # 6 hall sensor * pole_pairs
    axis.encoder.config.calib_range = 0.05
    axis.encoder.config.use_index = False
    odrv0.save_configuration()
    # axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION   
    # axis.encoder.config.pre_calibrated = True
    # odrv0.save_configuration()
    # axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    # axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    # axis.config.startup_closed_loop_control = AxisState.CLOSED_LOOP_CONTROL
    # odrv0.save_configuration()

if __name__ == "__main__":
    main()
