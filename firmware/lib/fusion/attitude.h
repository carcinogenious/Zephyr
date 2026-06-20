// Attitude estimator — Zephyr EDF TVC rocket (sensor fusion)
// Ported from SPARC. Complementary filter fusing the MPU6050 gyro (fast,
// immune to linear acceleration, but drift-prone) with the accelerometer's
// gravity vector (slow, noisy, but absolute) into signed pitch and roll. This
// is the attitude the TVC PID stabilizes and the angle the ToF slant range is
// tilt-corrected with to get true vertical height.
#pragma once

#include <Arduino.h>

namespace fusion {

struct Attitude {
    float pitch_deg;   // signed tilt of the board's X axis from level
    float roll_deg;    // signed tilt of the board's Y axis from level
};

// One complementary-filter step. Call once per control loop with accel in
// m/s², gyro in rad/s (bias-corrected), and dt in seconds. Holds filter state
// internally; the first call (or one after reset()) seeds straight from the
// accelerometer so it starts at the true angle instead of ramping from zero.
Attitude update(float ax, float ay, float az,
                float gx, float gy, float dt_s);

// Accelerometer-only tilt (degrees) from the gravity vector — no gyro, no filter
// state. The absolute, drift-free truth the filter's steady state must converge to.
// Exposed for bench diagnostics: print this next to update()'s fused output and
// they should agree at rest and track the same direction through a slow tilt. If
// they move OPPOSITE ways, that axis's gyro/accel signs disagree (see attitude.cpp).
Attitude accelOnly(float ax, float ay, float az);

// Forget the filter state; the next update() re-seeds from the accelerometer.
void reset();

}  // namespace fusion
