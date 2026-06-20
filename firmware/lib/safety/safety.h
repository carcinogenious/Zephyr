// safety.h — Zephyr hard failsafe + loop watchdog
//
// Two always-on, independent protections for the powered phase:
//
//  1. Hard failsafe — check(): while FLYING, returns a non-NONE Trip (and the
//     caller cuts the ESC to minimum) if ANY of:
//       • tilt exceeds cfg::SAFETY_MAX_TILT_DEG, or
//       • flight time exceeds cfg::FLIGHT_TIMEOUT_MS (the hop-duration cap), or
//       • a critical sensor (IMU or ToF) goes silent for longer than
//         cfg::SAFETY_SENSOR_TIMEOUT_MS.
//     This single rule defuses the common runaway cases. check() is a pure
//     decision — it has no side effects; the caller acts on the Trip.
//
//  2. Loop watchdog — init()/feed(): a monitor task on the OTHER core cuts the
//     ESC to minimum directly if the control loop stops calling feed() for
//     cfg::SAFETY_LOOP_WATCHDOG_MS. This covers a stalled/hung loop, where the
//     failsafe check above would never get a chance to run.
#pragma once

#include <cstdint>

namespace safety {

enum class Trip : uint8_t {
    NONE = 0,
    TILT,          // tilt exceeded the limit
    FLIGHT_TIME,   // exceeded the max hop duration
    SENSOR_FAULT,  // IMU or ToF stopped responding mid-flight
};

// Per-loop inputs for the in-flight failsafe check.
struct Inputs {
    bool     flying;         // sequencer is in FLYING — the failsafe only trips here
    float    tilt_deg;       // tilt off vertical (NAN when attitude unavailable)
    uint32_t flightTime_ms;  // elapsed time since launch
    bool     imuLive;        // a fresh IMU sample arrived this loop
    bool     tofLive;        // the ToF answered this loop
};

// Start the loop-watchdog monitor task. Call once in setup(), after throttle::init().
void init();

// Heartbeat — call once per control-loop iteration to keep the watchdog satisfied.
void feed();

// Evaluate the in-flight failsafe. Returns the trip reason (NONE if all clear).
// When not flying it resets the sensor-liveness timers and returns NONE.
Trip check(const Inputs& in);

// Human-readable trip reason, for logging / OLED.
const char* name(Trip t);

}  // namespace safety
