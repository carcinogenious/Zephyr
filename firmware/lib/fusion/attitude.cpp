#include "attitude.h"

#include <math.h>

namespace {
// Filter state, in radians.
float pitch_rad = 0.0f;
float roll_rad = 0.0f;
bool seeded = false;

// Complementary-filter weight on the gyro path. 0.98 trusts the gyro for the
// fast, thrust-immune short term and lets the accel trim ~2% of slow drift
// per step.
const float ALPHA = 0.98f;
}  // namespace

fusion::Attitude fusion::update(float ax, float ay, float az,
                                float gx, float gy, float dt_s) {
    // Absolute (drift-free) tilt from the gravity vector. With the board flat
    // (its +Z normal up along the thrust axis), at rest ax≈ay≈0 and az≈+g, so
    // both angles read ~0 and grow as the board leans.
    float pitchAccel = atan2f(ax, sqrtf(ay * ay + az * az));
    float rollAccel  = atan2f(ay, sqrtf(ax * ax + az * az));

    if (!seeded) {
        pitch_rad = pitchAccel;
        roll_rad = rollAccel;
        seeded = true;
        return {pitch_rad * RAD_TO_DEG, roll_rad * RAD_TO_DEG};
    }

    // Gyro pairing for a flat mount: rotation about Y tilts the X axis → gy
    // drives pitch; rotation about X tilts the Y axis → gx drives roll. If a
    // bench check shows an axis swapped or a sign inverted, these two rate
    // lines are the one place to fix it.
    pitch_rad = ALPHA * (pitch_rad + gy * dt_s) + (1.0f - ALPHA) * pitchAccel;
    roll_rad  = ALPHA * (roll_rad  + gx * dt_s) + (1.0f - ALPHA) * rollAccel;

    return {pitch_rad * RAD_TO_DEG, roll_rad * RAD_TO_DEG};
}

void fusion::reset() {
    seeded = false;
}
