// TVC nozzle actuator — Zephyr EDF TVC rocket
// Turns commanded nozzle deflections (degrees, per axis) into servo pulses for the
// two MG90S gimbal servos, driven via native LEDC (ledcAttachChannel, ch1/ch2,
// sharing the 50 Hz timer with the ESC — the proven 3-PWM path; see CLAUDE.md).
// Owns the safety-critical
// pieces: the ±cfg::TVC_CLAMP_DEG degree clamp (PRIMARY), the per-axis center µs,
// the per-axis/per-SIDE deg→µs slopes, the per-axis direction sign, and a hard µs
// backstop. The attitude PID produces the deflections; this turns them into motion.
#pragma once

#include <Arduino.h>

namespace tvc {

// Attach both servos on their explicit LEDC channels and center the nozzle (each
// axis to its own center µs). Call once in setup().
void init();

// Command the nozzle straight (0°, 0°) → each axis's center µs.
void center();

// Command deflections (degrees). Per axis: clamp to ±cfg::TVC_CLAMP_DEG, apply the
// per-axis direction sign, convert with the matching per-SIDE slope around the
// per-axis center µs, then apply the hard µs backstop before writing the servo.
void setDeflection(float pitchDeg, float rollDeg);

float pitchDeg();   // last commanded pitch deflection, after the clamp
float rollDeg();     // last commanded roll deflection,   after the clamp

uint16_t pitchMicros();  // last pulse written to the pitch servo (µs)
uint16_t rollMicros();    // last pulse written to the roll servo   (µs)

}  // namespace tvc
