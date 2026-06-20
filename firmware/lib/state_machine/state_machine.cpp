#include "state_machine.h"

#include <Arduino.h>

#include <zephyr_config.h>
#include <throttle.h>
#include <display.h>
#include <wifi_link.h>
#include <safety.h>
#include <battery.h>

namespace {
const float M_TO_IN = 39.3701f;  // meters → inches (the flight summary is in inches)

state_machine::Mode mode_ = state_machine::Mode::DISARMED;

// Per-flight tracking, owned by the sequencer.
unsigned long countdownStartMs = 0;
unsigned long flightStartMs = 0;
float peakVert_in = 0.0f;
float maxTilt_deg = 0.0f;

// End the flight: cut the ESC, publish + show the summary, and stand down to
// DISARMED. Shared by a normal landing, a brownout ramp completion, and the
// battery floor cut. Returns the flight duration in seconds (for logging).
float endFlight(unsigned long now) {
    float dur = (now - flightStartMs) / 1000.0f;
    throttle::disarm();
    mode_ = state_machine::Mode::DISARMED;
    wifi_link::publishSummary(peakVert_in, maxTilt_deg, dur);
    display::flightSummary(peakVert_in, dur, maxTilt_deg);
    return dur;
}
}  // namespace

void state_machine::init() {
    mode_ = Mode::DISARMED;
    peakVert_in = 0.0f;
    maxTilt_deg = 0.0f;
}

state_machine::Mode state_machine::update(const Inputs& in) {
    unsigned long now = millis();

    // Arm / launch / abort come from the WiFi control page only — there is no
    // physical button. wifi_link sets flags this loop polls (takeArm/takeLaunch/
    // takeAbort); the radio can never gate the control update, and the fan never
    // spins on boot.
    bool wantArm    = wifi_link::takeArm();
    bool wantLaunch = wifi_link::takeLaunch();
    bool wantAbort  = wifi_link::takeAbort();

    if (wantAbort) {
        throttle::disarm();
        mode_ = Mode::DISARMED;
        Serial.println(F("ABORT — throttle cut, idle"));
        display::message("ABORT", "idle");
    } else if (wantArm && mode_ == Mode::DISARMED) {
        Serial.println(F("ARMING ESC — FAN MUST BE CLAMPED"));
        display::message("ARMING", "fan clamped!");
        battery::resetLatch();  // re-evaluate the pack for this flight (post swap/charge)
        throttle::arm();
        mode_ = Mode::ARMED;
        Serial.println(F("ARMED — LAUNCH to fly"));
    } else if (wantLaunch && mode_ == Mode::ARMED) {
        countdownStartMs = now;
        mode_ = Mode::COUNTDOWN;
        Serial.println(F("COUNTDOWN — verifying sensors"));
    }

    // COUNTDOWN: re-verify arm + sensors continuously; spool only if still good.
    if (mode_ == Mode::COUNTDOWN) {
        if (!(throttle::armed() && in.tofOk && in.mpuOk)) {
            Serial.println(F("Sensor/arm check failed — abort"));
            throttle::disarm();
            mode_ = Mode::DISARMED;
        } else if (now - countdownStartMs >= cfg::LAUNCH_COUNTDOWN_MS) {
            // PHASE 1: open-loop manual throttle (the web slider). The closed-loop
            // profile (throttle::beginProfile) is Phase 2 and is not started here.
            flightStartMs = now;
            peakVert_in = 0.0f;
            maxTilt_deg = 0.0f;
            mode_ = Mode::FLYING;
            Serial.println(F("LAUNCH — manual throttle live, TVC active"));
        }
    }

    // Tilt off vertical (NAN until there's an attitude estimate). Computed once
    // and reused for both the hard failsafe and the flight summary.
    bool flying = (mode_ == Mode::FLYING);
    float tilt_deg = NAN;
    if (in.attReady) {
        tilt_deg = degrees(acosf(cosf(in.pitch_deg * DEG_TO_RAD) *
                                 cosf(in.roll_deg * DEG_TO_RAD)));
    }

    // ── Hard failsafe (lib/safety): tilt OR over-time OR sensor fault → cut ───
    // Evaluated every tick (it keeps its sensor timers fresh on the pad) but only
    // acts in flight. On a trip, cut the ESC immediately and stand down.
    safety::Inputs sf{};
    sf.flying        = flying;
    sf.tilt_deg      = tilt_deg;
    sf.flightTime_ms = flying ? (uint32_t)(now - flightStartMs) : 0;
    sf.imuLive       = in.imuLive;
    sf.tofLive       = in.tofLive;
    safety::Trip trip = safety::check(sf);
    if (flying && trip != safety::Trip::NONE) {
        throttle::disarm();
        mode_ = Mode::DISARMED;
        Serial.print(F("FAILSAFE ")); Serial.print(safety::name(trip));
        Serial.println(F(" — throttle cut"));
        display::message("FAILSAFE", safety::name(trip));
        return mode_;
    }

    // FLYING: run the profile on the ToF vertical; track peak height + max tilt.
    if (mode_ == Mode::FLYING) {
        // ── Battery brownout protection (lib/battery), latched ───────────────
        // FLOOR → hard cut and stand down. RAMP → arm a smooth powered ramp-down
        // (lib/throttle); TVC stays active (it lives in main), so the vehicle
        // settles under power. The ramp ends by returning Phase::LANDED below.
        battery::State bs = battery::state();
        if (bs == battery::State::CUT) {
            float dur = endFlight(now);
            Serial.print(F("BATT FLOOR ")); Serial.print(battery::volts(), 1);
            Serial.print(F("V — hard cut, ")); Serial.print(dur, 1);
            Serial.println(F(" s"));
            return mode_;
        }
        if (bs == battery::State::RAMP && throttle::beginRampDown()) {
            Serial.print(F("BATT LOW ")); Serial.print(battery::volts(), 1);
            Serial.println(F("V — throttle ramp-down"));
        }

        if (!isnan(tilt_deg) && tilt_deg > maxTilt_deg) maxTilt_deg = tilt_deg;
        if (!isnan(in.vertical_m) && in.vertical_m * M_TO_IN > peakVert_in) {
            peakVert_in = in.vertical_m * M_TO_IN;
        }
        // PHASE 1: command the web-slider throttle (open-loop, voltage-compensated
        // + slew-limited inside lib/throttle). A low-battery ramp-down overrides the
        // slider and returns LANDED when it completes → end the flight.
        if (throttle::setManual(in.manualThrottle, in.dt_s) == throttle::Phase::LANDED) {
            float dur = endFlight(now);
            Serial.print(F("LANDED — peak ")); Serial.print(peakVert_in, 1);
            Serial.print(F(" in, maxTilt ")); Serial.print(maxTilt_deg, 1);
            Serial.print(F(" deg, ")); Serial.print(dur, 1); Serial.println(F(" s"));
        }
    }

    return mode_;
}

state_machine::Mode state_machine::mode() { return mode_; }
