#include "pid.h"

pid::Controller::Controller(const Gains& gains, float iClampOut)
    : gains_(gains), iClampOut_(iClampOut),
      integral_(0.0f), prevError_(0.0f), hasPrev_(false) {}

float pid::Controller::update(float setpoint, float measurement, float dt_s) {
    float error = setpoint - measurement;

    float p = gains_.kp * error;

    // Integrate, then clamp so the integral CONTRIBUTION (ki·integral) stays
    // within ±iClampOut. Back-calculate the accumulator from the clamped term
    // so the bound holds even if ki is retuned, and so the integral doesn't
    // keep winding past what it can output.
    integral_ += error * dt_s;
    float i = gains_.ki * integral_;
    if (i > iClampOut_) {
        i = iClampOut_;
        if (gains_.ki != 0.0f) integral_ = i / gains_.ki;
    } else if (i < -iClampOut_) {
        i = -iClampOut_;
        if (gains_.ki != 0.0f) integral_ = i / gains_.ki;
    }

    // Derivative on error. Skipped on the first call (no history) and when dt is
    // non-positive, so a stale/zero dt can't spike the output.
    float d = 0.0f;
    if (hasPrev_ && dt_s > 0.0f) {
        d = gains_.kd * (error - prevError_) / dt_s;
    }
    prevError_ = error;
    hasPrev_ = true;

    return p + i + d;
}

float pid::Controller::update(float setpoint, float measurement, float measRate,
                              float dt_s, float outClamp) {
    float error = setpoint - measurement;
    float p = gains_.kp * error;
    // Derivative on the measurement rate (gyro): for a constant setpoint
    // d(error)/dt = -d(measurement)/dt, so this damps motion without setpoint kick.
    float d = -gains_.kd * measRate;

    // Tentative output with the CURRENT integral, before deciding whether to integrate.
    float i = gains_.ki * integral_;
    float out = p + i + d;

    // Conditional integration (anti-windup): integrate only when NOT pushing an
    // already-saturated output further into the clamp. Frozen otherwise.
    bool pushHigh = out >= outClamp  && error > 0.0f;
    bool pushLow  = out <= -outClamp && error < 0.0f;
    if (!pushHigh && !pushLow && dt_s > 0.0f) {
        integral_ += error * dt_s;
        i = gains_.ki * integral_;
        if (i > iClampOut_) {
            i = iClampOut_;
            if (gains_.ki != 0.0f) integral_ = i / gains_.ki;
        } else if (i < -iClampOut_) {
            i = -iClampOut_;
            if (gains_.ki != 0.0f) integral_ = i / gains_.ki;
        }
        out = p + i + d;
    }

    if (out > outClamp) out = outClamp;
    else if (out < -outClamp) out = -outClamp;
    lastP_ = p; lastI_ = i; lastD_ = d;   // diagnostics
    return out;
}

void pid::Controller::reset() {
    integral_ = 0.0f;
    prevError_ = 0.0f;
    hasPrev_ = false;
}
