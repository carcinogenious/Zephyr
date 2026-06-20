// Throttle / ESC controller — Zephyr EDF TVC rocket
// Owns the EDF's ESC PWM signal (1000–2000 µs via native LEDC ch0) and the canned
// vertical flight profile: ESC arming, then a closed-loop ascend → hover →
// descend trajectory. The altitude PID output rides on top of the hover
// feed-forward (cfg::THROTTLE_HOVER), the command is voltage-compensated (scaled by
// (V_REF/V_now)^EXP to hold thrust as the pack sags — matters most for open-loop),
// and throttle changes are slew-limited, so ascent and descent are smooth/stable.
// Battery protection (ramp-down / floor cut) bypasses compensation and always wins.
// Lateral stability is NOT here — that is the attitude PID + lib/tvc.
//
// SAFETY: arming spins nothing, but update() WILL drive the fan. Bench-test only
// with the EDF clamped (cfg comments / build-guide Part A Step 3).
#pragma once

#include <Arduino.h>

namespace throttle {

enum class Phase : uint8_t { IDLE, ASCEND, HOVER, DESCEND, LANDED, MANUAL };

// Attach the ESC at minimum (disarmed, fan off). Call once in setup().
void init();

// Run the ESC arming sequence: hold minimum for cfg::ESC_ARM_MS so the ESC
// arms (it beeps). Blocking (~2 s). Returns true once armed. FAN CLAMPED.
bool arm();

// Cut to minimum and mark disarmed. Safe to call any time.
void disarm();

// BENCH TEST ONLY: write a raw throttle fraction (0..1) straight to the ESC,
// bypassing the flight profile/PID, voltage compensation, and the arm gate. For
// powered bench checks with the FAN CLAMPED (e.g. voltage-under-load). Call arm()
// first so the ESC is live. Not used by the flight path.
void writeTest(float frac);

// PHASE 1 (manual open-loop throttle): command a base throttle fraction (0..1,
// e.g. the web slider) — voltage-compensated, THROTTLE_MAX-capped, and slew-limited.
// Held at minimum until arm(). A low-battery ramp-down overrides it (protection
// always wins) and returns Phase::LANDED when the ramp completes; otherwise returns
// Phase::MANUAL. Replaces update() while flying open-loop; closed-loop is Phase 2.
Phase setManual(float baseFrac, float dt_s);

// Begin a smooth brownout ramp-down: linearly ease the current throttle to zero
// over cfg::BATT_RAMP_DOWN_MS, then hold at minimum. Pulses keep flowing (the ESC
// stays armed) and TVC is unaffected, so the vehicle settles under power instead
// of dropping. update() drives the ramp and returns Phase::LANDED once it
// completes. Triggered by lib/battery via the sequencer on a low pack; the hard
// floor cut uses disarm() instead.
// Returns true only on the call that STARTS the ramp; false if already ramping
// (idempotent), so the caller can log the trip once.
bool beginRampDown();

// Start the ascend→hover→descend profile from the current point. Resets the
// altitude PID and timers. Requires arm() first.
void beginProfile();

// Step the profile once: advances the altitude setpoint for the current phase,
// runs the altitude PID against the measured vertical height (meters AGL — pass
// the ToF tilt-corrected vertical), slew-limits, and WRITES the ESC. No-op
// (holds minimum) until armed + beginProfile().
// Returns the current phase; reaches LANDED when the descent completes.
Phase update(float altitude_m, float dt_s);

Phase phase();
float fraction();          // last commanded throttle fraction 0..1 (telemetry)
float targetAltitude_m();  // current altitude setpoint, m (telemetry)
bool  armed();

}  // namespace throttle
