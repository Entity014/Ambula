# Ambula Motor (Configuration)

```python
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    axis.encoder.config.pre_calibrated = True
    odrv0.save_configuration()
    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    odrv0.save_configuration()
```
