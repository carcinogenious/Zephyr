// PID controller — Zephyr EDF TVC rocket
// Generic single-axis PID with integral anti-windup. Unit-agnostic: the caller
// assigns meaning. For the TVC attitude loop, setpoint/measurement are degrees
// of tilt (setpoint 0 = vertical) and the output is degrees of nozzle deflection.
#pragma once

#include <Arduino.h>

namespace pid {

struct Gains {
    float kp;
    float ki;
    float kd;
};

class Controller {
public:
    // iClampOut bounds the integral TERM's contribution to the output (in the
    // output's units), which is what limits wind-up.
    Controller(const Gains& gains, float iClampOut);

    // One step. setpoint and measurement share units; dt_s in seconds (> 0).
    // Returns the control output. Derivative is taken on the error. (Altitude loop.)
    float update(float setpoint, float measurement, float dt_s);

    // Attitude-loop step. Derivative is taken on the MEASUREMENT RATE (e.g. the gyro
    // rate, units/s) — cleaner than differencing the angle and free of setpoint
    // kick. The output is clamped to ±outClamp, and the integral uses conditional
    // integration: it stops accumulating in the direction that would push an already
    // saturated output further out, so it cannot wind up against the clamp.
    float update(float setpoint, float measurement, float measRate,
                 float dt_s, float outClamp);

    // Clear integral + derivative history (e.g. on state change / disarm).
    void reset();

    void setGains(const Gains& gains) { gains_ = gains; }

    // Last step's term breakdown (diagnostics only) — what P, I and D each
    // contributed to the most recent update() output, in the output's units.
    float lastP() const { return lastP_; }
    float lastI() const { return lastI_; }
    float lastD() const { return lastD_; }

private:
    Gains gains_;
    float iClampOut_;
    float integral_;     // accumulated error·dt
    float prevError_;
    bool  hasPrev_;
    float lastP_ = 0.0f, lastI_ = 0.0f, lastD_ = 0.0f;  // diagnostics
};

}  // namespace pid
