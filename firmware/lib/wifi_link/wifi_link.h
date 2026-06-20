// WiFi link — Zephyr EDF TVC rocket
// A convenience layer ONLY: (1) a pre-coded launch/arm/abort trigger and (2)
// telemetry downlink. It must never be able to affect onboard flight logic.
//
// The ESP32 runs as its own access point and serves a control page. The web
// server lives in a separate FreeRTOS task (core 0); the control loop runs in
// loop() (core 1). The two share only a telemetry snapshot and a few command
// flags. The control loop NEVER calls into the network and NEVER waits on it:
// it reads flags (non-blocking) and writes the snapshot (a brief copy). A page
// refresh, a full buffer, or a dropped link cannot stall the PID loop.
#pragma once

#include <Arduino.h>

namespace wifi_link {

// Latest flight snapshot for the downlink. Filled by the control loop via
// publish(); read by the web task. Fire-and-forget — stale frames are fine.
struct Telemetry {
    float vertical_in;     // ToF tilt-corrected vertical height (inches)
    float setpoint_in;     // altitude setpoint (inches)
    float pitch_deg, roll_deg;
    float defPitch_deg, defRoll_deg;  // commanded nozzle deflections (pre-clamp)
    float throttlePct;
    float battV;
    uint8_t phase;         // throttle::Phase
    uint8_t flightMode;    // 0 DISARMED, 1 ARMED, 2 COUNTDOWN, 3 FLYING
    bool bmpOk, mpuOk, tofOk;
};

// Start the SoftAP + web server task. Call once in setup(). Prints the AP name
// and URL to Serial.
void init();

// Copy the latest telemetry into shared state (called from the control loop).
// Brief critical section only — never touches the radio, never blocks.
void publish(const Telemetry& t);

// Post-landing summary the page surfaces for pasting into docs/flight_logs/.
void publishSummary(float peakAlt_in, float maxTilt_deg, float duration_s);

// Command events from the page, consumed by the control loop. Each returns true
// at most once per press (edge), so the loop just polls them every cycle.
bool takeArm();
bool takeLaunch();
bool takeAbort();

// Latest manual throttle command from the page slider, 0..1 (Phase-1 open-loop).
// A level, not an edge: the loop reads the latest value every cycle. Reset to 0
// server-side on ARM and ABORT so each flight starts at zero throttle.
float manualThrottle();

// Live attitude-PID gains, edited from the page (per-axis Kp/Ki/Kd). The control
// loop SEEDS these once with the config defaults, then reads them every cycle and
// applies them to its controllers — so the page tunes gains without a reflash.
// (Pitch and roll are independent; gains reset to the config defaults on reboot.)
struct PidGains { float kp, ki, kd; };
void seedGains(const PidGains& pitch, const PidGains& roll);
PidGains pitchGains();
PidGains rollGains();

}  // namespace wifi_link
