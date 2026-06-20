// state_machine.h — Zephyr flight sequencer
//
// The high-level "what is the rocket doing" machine: DISARMED → ARMED →
// COUNTDOWN → FLYING, with abort / land / timeout returning to DISARMED. It owns
// the arm/launch/abort events (from the WiFi page — no physical button), the ESC
// arming and launch gating, and the in-flight summary (peak height, max tilt,
// duration).
//
// It does NOT decide the throttle level itself. PHASE 1 (now): FLYING delegates to
// throttle::setManual() with the web slider value each tick (open-loop, voltage-
// compensated). PHASE 2 (later): the closed-loop throttle::update() profile. Either
// way, when the throttle reports LANDED the sequencer cuts the motor and returns to
// DISARMED. Battery/safety protection is enforced here, ahead of the throttle write.
//
// main.cpp gathers this loop's Inputs and calls update() once per control tick;
// it never touches the flight state directly.
#pragma once

#include <cstdint>

namespace state_machine {

enum class Mode : uint8_t {
    DISARMED  = 0,  // on the pad, ESC not armed — the fan cannot spin
    ARMED     = 1,  // ESC armed (idle), waiting for the launch command
    COUNTDOWN = 2,  // re-verifying arm + sensors before spool-up
    FLYING    = 3,  // running the throttle profile under active TVC
};

// Everything the sequencer needs from the rest of the system this loop. main.cpp
// fills this in from the fused attitude, the ToF vertical, and sensor health.
struct Inputs {
    float vertical_m;  // tilt-corrected vertical height (m), NAN if unavailable
    float pitch_deg;   // fused attitude (deg) — valid only when attReady
    float roll_deg;
    bool  attReady;    // attitude estimate is valid this loop
    bool  mpuOk;       // IMU healthy (latched at boot)
    bool  tofOk;       // ToF healthy (latched at boot)
    bool  imuLive;     // fresh IMU sample THIS loop (for the in-flight failsafe)
    bool  tofLive;     // ToF answered THIS loop (for the in-flight failsafe)
    float dt_s;        // control-loop dt, seconds
    float manualThrottle;  // web-slider throttle 0..1 (Phase-1 open-loop command)
};

// Reset the sequencer to DISARMED. Call once in setup(), after throttle::init().
// Arm/launch/abort arrive over the WiFi page (lib/wifi_link) — no local button.
void init();

// Run one tick of the sequencer (events → transitions → throttle hand-off).
// Returns the current (possibly just-changed) mode.
Mode update(const Inputs& in);

// Current mode, for telemetry. Does not advance the machine.
Mode mode();

}  // namespace state_machine
