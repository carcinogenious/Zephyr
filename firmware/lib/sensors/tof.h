// VL53L1X time-of-flight rangefinder driver — Zephyr EDF TVC rocket
// Ported from SPARC (same sensor, same Pololu VL53L1X library). Mounted facing
// downward; provides the precise low-altitude reading used by the fusion filter
// when the rocket is close to the ground (< 4 m) and roughly upright.
//
// Named `tof.h` (not `vl53l1x.h`) to avoid a case-insensitive filesystem
// collision with the Pololu library's `VL53L1X.h`.
#pragma once

#include <Arduino.h>
#include <zephyr_config.h>

namespace tof {

struct Reading {
    float range_m;          // slant range RISEN from launch (raw range minus the boot baseline)
    float vertical_m;       // tilt-corrected height above launch (range_m * cos(tilt))
    float tilt_rad;         // tilt angle used for the correction (0 = perfectly upright)
    bool ok;                 // tilt-corrected vertical_m is valid — needs a usable IMU
                             //   gravity vector, so this is NOT a sensor-health signal
    bool sensor_ok;          // the ToF itself is responding on I2C this cycle. Fully
                             //   independent of the IMU and of range quality — use this
                             //   (not ok) for the status display. Self-clears if the
                             //   sensor stops answering.
};

// Initializes the VL53L1X on the shared I2C bus in long-range continuous
// mode (~4 m max) at the 50 Hz control-loop rate. Returns false if the
// sensor is not found.
bool init(uint8_t sdaPin = cfg::PIN_I2C_SDA, uint8_t sclPin = cfg::PIN_I2C_SCL);

// Captures the resting slant range as the launch zero (the sensor's standoff
// above the pad). Samples until the range stabilizes or timeoutMs elapses —
// hold the rocket still. Returns true if it stabilized; on timeout it still
// sets a best-effort baseline and returns false. Call once after init().
bool captureBaseline(unsigned long timeoutMs = 10000);

// Reads the latest slant range and combines it with the IMU's gravity
// vector (accel_x/y/z, m/s²) to compute the true vertical altitude.
// The rangefinder points along the rocket's body z-axis; tilt is the
// angle between that axis and gravity. Non-blocking: if no new ToF
// sample is ready, returns the previous slant range with a fresh
// tilt correction.
Reading read(float accel_x, float accel_y, float accel_z);

}  // namespace tof
