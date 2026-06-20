#include "tvc.h"

#include <Arduino.h>
#include <zephyr_config.h>

// Servos on NATIVE LEDC (ledcAttachChannel, explicit channels 1/2, sharing the
// 50 Hz timer with the ESC on ch0) — the PROVEN 3-PWM coexistence path on this
// S3 + core 3.x. ESP32Servo's 3-instance MCPWM combo corrupted the ESC (see
// CLAUDE.md). lib/tvc commands the target pulse and lets the analog servo slew to
// it natively (no software interpolation, write-on-change) for smooth vectoring.

namespace {
float lastPitch = 0.0f;    // last commanded deflection (deg, after clamp)
float lastRoll   = 0.0f;
uint16_t lastPitchUs = 0;  // last pulse written to each servo (µs)
uint16_t lastRollUs   = 0;

// µs → LEDC duty for the configured resolution over the 50 Hz (PWM_PERIOD_US) frame.
uint32_t usToDuty(uint16_t us) {
    return (uint32_t)(((uint64_t)us << cfg::PWM_RESOLUTION_BITS) / cfg::PWM_PERIOD_US);
}

// One axis: clamp degrees (PRIMARY limit), apply direction sign, convert with the
// matching per-SIDE slope around the per-axis center, then apply the hard µs
// backstop (mechanical-safe) so a bad value can't drive the servo into its bind.
uint16_t axisToMicros(float deg, uint16_t centerUs, int8_t dir,
                      float slopePos, float slopeNeg,
                      uint16_t usMin, uint16_t usMax) {
    if (deg >  cfg::TVC_CLAMP_DEG) deg =  cfg::TVC_CLAMP_DEG;
    if (deg < -cfg::TVC_CLAMP_DEG) deg = -cfg::TVC_CLAMP_DEG;
    float d = dir * deg;                                  // logical → physical
    float us = (d >= 0.0f) ? centerUs + d * slopePos
                           : centerUs + d * slopeNeg;     // d<0 subtracts via slopeNeg
    if (us < usMin) us = usMin;
    if (us > usMax) us = usMax;
    return (uint16_t)(us + 0.5f);
}
}  // namespace

void tvc::init() {
    // Attach each servo's LEDC channel ONCE (re-attaching returns false). Both
    // share the 50 Hz timer with the ESC on ch0 — the proven 3-PWM arrangement.
    ledcAttachChannel(cfg::PIN_SERVO_PITCH, cfg::SERVO_PWM_HZ,
                      cfg::PWM_RESOLUTION_BITS, cfg::LEDC_CH_PITCH);
    ledcAttachChannel(cfg::PIN_SERVO_ROLL, cfg::SERVO_PWM_HZ,
                      cfg::PWM_RESOLUTION_BITS, cfg::LEDC_CH_ROLL);
    center();  // nozzle straight (per-axis center µs) before the control loop runs
}

void tvc::center() {
    setDeflection(0.0f, 0.0f);
}

void tvc::setDeflection(float pitchDeg, float rollDeg) {
    lastPitch = constrain(pitchDeg, -cfg::TVC_CLAMP_DEG, cfg::TVC_CLAMP_DEG);
    lastRoll   = constrain(rollDeg,   -cfg::TVC_CLAMP_DEG, cfg::TVC_CLAMP_DEG);

    uint16_t pUs = axisToMicros(pitchDeg, cfg::TVC_PITCH_CENTER_US, cfg::TVC_PITCH_DIR,
                                cfg::TVC_PITCH_US_PER_DEG_POS, cfg::TVC_PITCH_US_PER_DEG_NEG,
                                cfg::TVC_PITCH_US_MIN, cfg::TVC_PITCH_US_MAX);
    uint16_t rUs = axisToMicros(rollDeg, cfg::TVC_ROLL_CENTER_US, cfg::TVC_ROLL_DIR,
                                cfg::TVC_ROLL_US_PER_DEG_POS, cfg::TVC_ROLL_US_PER_DEG_NEG,
                                cfg::TVC_ROLL_US_MIN, cfg::TVC_ROLL_US_MAX);

    // Write only when the target µs actually changed — a HELD servo is written
    // once and left to the LEDC hardware, not hammered at the loop rate.
    // (The dominant jitter fix is still hardware: a 470µF+ cap across the servo
    // 5V/GND rail + a stiff 5V supply + common ground.)
    if (pUs != lastPitchUs) { ledcWrite(cfg::PIN_SERVO_PITCH, usToDuty(pUs)); lastPitchUs = pUs; }
    if (rUs != lastRollUs)   { ledcWrite(cfg::PIN_SERVO_ROLL,   usToDuty(rUs)); lastRollUs   = rUs; }
}

float tvc::pitchDeg() { return lastPitch; }
float tvc::rollDeg() { return lastRoll; }
uint16_t tvc::pitchMicros() { return lastPitchUs; }
uint16_t tvc::rollMicros() { return lastRollUs; }
