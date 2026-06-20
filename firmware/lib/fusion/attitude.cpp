#include "attitude.h"

#include <math.h>

#include <zephyr_config.h>

namespace {
// Filter state, in radians.
float pitch_rad = 0.0f;
float roll_rad = 0.0f;
bool seeded = false;

// Complementary-filter weight on the gyro path (cfg::FUSION_ALPHA, 0.98): trusts
// the gyro for the fast, thrust-immune short term and lets the accel trim ~2% of
// slow drift per step.
const float ALPHA = cfg::FUSION_ALPHA;

// Accelerometer-only tilt (radians) from the gravity vector. With the board flat
// (its +Z normal up along the thrust axis), at rest ax≈ay≈0 and az≈+g, so both
// angles read ~0 and grow as the board leans. Zephyr's MPU is mounted so the
// gimbal PITCH axis is rotation about the board X axis (tilts the Y reading →
// pitch from ay); ROLL is rotation about Y (→ roll from ax).
//
// SIGN: each accel angle must increase in the SAME direction its gyro mate
// integrates, or the complementary filter's transient (gyro) and steady (accel)
// states fight and the fused angle flips sign — a reversed, diverging estimate.
// For a +rotation about Y the gravity math gives ax=-g·sinθ, so the roll term
// carries a leading minus to read +θ and agree with +gy (the pitch/ay term
// already agrees with +gx and needs none).
void accelAnglesRad(float ax, float ay, float az, float& pitch, float& roll) {
    pitch = atan2f(ay, sqrtf(ax * ax + az * az));
    roll  = atan2f(-ax, sqrtf(ay * ay + az * az));
}
}  // namespace

fusion::Attitude fusion::update(float ax, float ay, float az,
                                float gx, float gy, float dt_s) {
    float pitchAccel, rollAccel;
    accelAnglesRad(ax, ay, az, pitchAccel, rollAccel);

    if (!seeded) {
        pitch_rad = pitchAccel;
        roll_rad = rollAccel;
        seeded = true;
        return {pitch_rad * RAD_TO_DEG, roll_rad * RAD_TO_DEG};
    }

    // Gyro pairing for Zephyr's mount: PITCH is rotation about X → gx integrates
    // pitch; ROLL is rotation about Y → gy integrates roll. (This is 90° from a
    // "flat" board where gy→pitch; Zephyr's IMU sits rotated from SPARC's.) If a
    // bench check shows an axis swapped or a sign inverted, these two rate lines +
    // the two accel lines above are the one place to fix it.
    pitch_rad = ALPHA * (pitch_rad + gx * dt_s) + (1.0f - ALPHA) * pitchAccel;
    roll_rad  = ALPHA * (roll_rad  + gy * dt_s) + (1.0f - ALPHA) * rollAccel;

    return {pitch_rad * RAD_TO_DEG, roll_rad * RAD_TO_DEG};
}

fusion::Attitude fusion::accelOnly(float ax, float ay, float az) {
    float p, r;
    accelAnglesRad(ax, ay, az, p, r);
    return {p * RAD_TO_DEG, r * RAD_TO_DEG};
}

void fusion::reset() {
    seeded = false;
}
