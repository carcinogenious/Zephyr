#include "throttle.h"

#include <Arduino.h>
#include <math.h>
#include <battery.h>
#include <pid.h>
#include <zephyr_config.h>

// ESC on NATIVE LEDC (ledcAttachChannel, channel 0, sharing the 50 Hz timer with
// the two servos on ch1/ch2) — the proven 3-PWM coexistence path on this S3 + core
// 3.x. ESP32Servo's 3-instance MCPWM combo corrupted this output (see CLAUDE.md).

namespace {
bool isArmed = false;

// µs → LEDC duty for the configured resolution over the 50 Hz (PWM_PERIOD_US) frame.
uint32_t usToDuty(uint16_t us) {
    return (uint32_t)(((uint64_t)us << cfg::PWM_RESOLUTION_BITS) / cfg::PWM_PERIOD_US);
}

// Write a raw µs pulse to the ESC's LEDC channel.
void escWriteUs(uint16_t us) {
    ledcWrite(cfg::PIN_ESC, usToDuty(us));
}

throttle::Phase phase_ = throttle::Phase::IDLE;
float setpoint_m = 0.0f;        // current altitude setpoint, ramped per phase
float lastFrac = 0.0f;          // last commanded throttle fraction (for slew limit)
unsigned long hoverStartMs = 0; // millis() when HOVER began

// Brownout ramp-down (low battery): ease lastFrac→0 over cfg::BATT_RAMP_DOWN_MS.
bool brownout_ = false;
unsigned long rampStartMs = 0;
float rampStartFrac = 0.0f;     // throttle fraction when the ramp began

// Altitude PID: output is a throttle fraction added on top of the hover
// feed-forward, so it only trims the deviation from the setpoint.
pid::Controller altPid({cfg::KP_ALT, cfg::KI_ALT, cfg::KD_ALT}, cfg::ALT_I_CLAMP);

// Write a throttle fraction (0..1) to the ESC as a 1000–2000 µs pulse.
void writeFraction(float f) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    lastFrac = f;
    escWriteUs(cfg::ESC_MIN_US +
               (uint16_t)(f * (cfg::ESC_MAX_US - cfg::ESC_MIN_US)));
}

// Voltage compensation: hold thrust ~constant as the pack sags. The throttle
// FRACTION already represents the command ABOVE ESC_MIN (the zero-thrust point), so
// scaling it by (V_REF/V_now)^EXP is exactly "ESC_MIN + (cmd-ESC_MIN)·factor".
// Reads the LIVE (under-load) pack voltage. The caller clamps to THROTTLE_MAX; if
// comp wants more, the pack is depleted and battery protection (RAMP/FLOOR) — which
// is evaluated by the sequencer BEFORE this runs — overrides it.
float voltageCompensate(float baseFrac) {
    float v = battery::volts();
    if (v < 1.0f) return baseFrac;   // no/implausible reading → don't scale blindly
    return baseFrac * powf(cfg::THROTTLE_V_REF_V / v, cfg::THROTTLE_VCOMP_EXP);
}

// Commit a BASE throttle fraction (the desired command before compensation):
// voltage-compensate it, cap at THROTTLE_MAX, slew-limit vs the last command so it
// can't step abruptly, then write. Shared by the closed-loop profile and the manual
// open-loop path.
void commitTarget(float baseFrac, float dt_s) {
    float target = voltageCompensate(baseFrac);
    if (target < 0.0f) target = 0.0f;
    if (target > cfg::THROTTLE_MAX) target = cfg::THROTTLE_MAX;
    float maxStep = cfg::THROTTLE_RAMP_PER_S * dt_s;
    if (target > lastFrac + maxStep) target = lastFrac + maxStep;
    if (target < lastFrac - maxStep) target = lastFrac - maxStep;
    writeFraction(target);
}

// Brownout ramp-down step (low battery): linearly ease the throttle from where it
// was to zero over BATT_RAMP_DOWN_MS so the vehicle settles under power, then report
// LANDED. Protection — bypasses compensation and never boosts. TVC keeps running.
throttle::Phase brownoutStep() {
    float k = 1.0f - (float)(millis() - rampStartMs) / cfg::BATT_RAMP_DOWN_MS;
    if (k <= 0.0f) { k = 0.0f; phase_ = throttle::Phase::LANDED; }
    writeFraction(rampStartFrac * k);
    return phase_;
}
}  // namespace

void throttle::init() {
    // Attach the ESC's LEDC channel ONCE (re-attaching returns false); shares the
    // 50 Hz timer with the servos on ch1/ch2. Start at minimum (disarmed, fan off).
    ledcAttachChannel(cfg::PIN_ESC, cfg::SERVO_PWM_HZ,
                      cfg::PWM_RESOLUTION_BITS, cfg::LEDC_CH_ESC);
    escWriteUs(cfg::ESC_MIN_US);
    isArmed = false;
    phase_ = Phase::IDLE;
    lastFrac = 0.0f;
    brownout_ = false;
}

bool throttle::arm() {
    // Hold minimum so the ESC sees a valid idle and arms (it beeps).
    escWriteUs(cfg::ESC_ARM_US);
    delay(cfg::ESC_ARM_MS);
    isArmed = true;
    return true;
}

void throttle::disarm() {
    escWriteUs(cfg::ESC_MIN_US);
    lastFrac = 0.0f;
    isArmed = false;
    phase_ = Phase::IDLE;
    brownout_ = false;
}

void throttle::writeTest(float frac) {
    writeFraction(frac);  // direct, no slew/PID; lastFrac updates so telemetry shows it
}

bool throttle::beginRampDown() {
    if (brownout_) return false;    // already ramping — keep the original start point
    brownout_ = true;
    rampStartMs = millis();
    rampStartFrac = lastFrac;       // ease from whatever we're currently commanding
    return true;
}

void throttle::beginProfile() {
    altPid.reset();
    setpoint_m = 0.0f;
    phase_ = Phase::ASCEND;
}

throttle::Phase throttle::update(float altitude_m, float dt_s) {
    if (!isArmed || phase_ == Phase::IDLE) {
        escWriteUs(cfg::ESC_MIN_US);
        return phase_;
    }

    // Brownout ramp-down (low battery) overrides the profile/PID — protection wins.
    if (brownout_) return brownoutStep();

    // Advance the altitude setpoint for the current phase. Ramping the setpoint
    // (rather than commanding the target directly) is what bounds the climb/
    // descent rate and keeps ascent + descent smooth.
    switch (phase_) {
        case Phase::ASCEND:
            setpoint_m += cfg::ASCENT_RATE_MS * dt_s;
            if (setpoint_m >= cfg::ALT_TARGET_M) {
                setpoint_m = cfg::ALT_TARGET_M;
                phase_ = Phase::HOVER;
                hoverStartMs = millis();
            }
            break;
        case Phase::HOVER:
            setpoint_m = cfg::ALT_TARGET_M;
            if (millis() - hoverStartMs >= cfg::HOVER_MS) {
                phase_ = Phase::DESCEND;
            }
            break;
        case Phase::DESCEND:
            setpoint_m -= cfg::DESCENT_RATE_MS * dt_s;
            if (setpoint_m <= cfg::ALT_LANDED_M || altitude_m <= cfg::ALT_LANDED_M) {
                setpoint_m = 0.0f;
                phase_ = Phase::LANDED;
            }
            break;
        case Phase::LANDED:
        case Phase::IDLE:
            break;
    }

    // Once landed, cut the motor and hold it cut.
    if (phase_ == Phase::LANDED) {
        writeFraction(0.0f);
        return phase_;
    }

    // Hover feed-forward + PID trim → voltage-compensate, cap, slew-limit, write.
    commitTarget(cfg::THROTTLE_HOVER + altPid.update(setpoint_m, altitude_m, dt_s), dt_s);
    return phase_;
}

throttle::Phase throttle::setManual(float baseFrac, float dt_s) {
    if (!isArmed) {                       // arming gate — slider can't spin a disarmed fan
        escWriteUs(cfg::ESC_MIN_US);
        return phase_;
    }
    if (brownout_) return brownoutStep(); // low-battery protection wins over the slider
    phase_ = Phase::MANUAL;
    commitTarget(baseFrac, dt_s);         // voltage-compensate + cap + slew + write
    return phase_;
}

throttle::Phase throttle::phase() { return phase_; }
float throttle::fraction() { return lastFrac; }
float throttle::targetAltitude_m() { return setpoint_m; }
bool throttle::armed() { return isArmed; }
