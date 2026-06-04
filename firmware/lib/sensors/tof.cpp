#include "tof.h"

#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>

namespace {
::VL53L1X sensor;
float lastRange_m = NAN;
unsigned long lastSampleMs = 0;        // millis() of the last successful I2C transaction
bool everSampled = false;              // have we ever heard from the sensor at all?
const unsigned long TOF_STALE_MS = 200; // ~10 missed 20 ms samples → treat as not responding

// Resting slant range captured at boot — the sensor's standoff above the pad.
// Subtracted from every reading so range_m / vertical_m are height RISEN from
// launch (~0 at rest), mirroring the BMP388 baseline.
float baselineRange_m = 0.0f;

// Stability gate for the boot capture: settled once the spread (max - min)
// across the sample window stays under this, in INCHES.
const float STABLE_SPREAD_IN = 6.0f;
}  // namespace

bool tof::init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);

    sensor.setBus(&Wire);
    sensor.setTimeout(100);
    if (!sensor.init()) {
        return false;
    }

    // Long mode reaches the full ~4 m we care about for low-altitude fusion.
    // 20 ms timing budget + 20 ms inter-measurement period gives the 50 Hz
    // sample rate the control loop expects.
    sensor.setDistanceMode(::VL53L1X::Long);
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous(20);
    return true;
}

bool tof::captureBaseline(unsigned long timeoutMs) {
    const int WINDOW = 10;                   // ~1 s of samples at 10 Hz
    const unsigned long PERIOD_MS = 100;
    const float stableSpread_m = STABLE_SPREAD_IN / 39.3701f;

    float win[WINDOW];
    int count = 0;
    int idx = 0;
    unsigned long start = millis();

    while (millis() - start < timeoutMs) {
        if (sensor.dataReady()) {
            uint16_t mm = sensor.read(false);
            if (!sensor.timeoutOccurred() &&
                sensor.ranging_data.range_status == ::VL53L1X::RangeValid) {
                win[idx] = mm / 1000.0f;
                idx = (idx + 1) % WINDOW;
                if (count < WINDOW) count++;

                // Window full + spread under threshold → the standoff reading
                // has settled, lock it as the launch zero.
                if (count == WINDOW) {
                    float mn = win[0], mx = win[0], sum = 0.0f;
                    for (int i = 0; i < WINDOW; i++) {
                        if (win[i] < mn) mn = win[i];
                        if (win[i] > mx) mx = win[i];
                        sum += win[i];
                    }
                    if (mx - mn < stableSpread_m) {
                        baselineRange_m = sum / WINDOW;
                        return true;
                    }
                }
            }
        }
        delay(PERIOD_MS);
    }

    // Timed out without settling. Set a best-effort baseline from whatever we
    // gathered so range_m is still launch-relative, and report the failure.
    if (count > 0) {
        float sum = 0.0f;
        for (int i = 0; i < count; i++) sum += win[i];
        baselineRange_m = sum / count;
    }
    return false;
}

tof::Reading tof::read(float accel_x, float accel_y, float accel_z) {
    Reading r{};

    // Non-blocking: only consume a new sample if one is ready, otherwise
    // recompute the tilt correction on the last-known slant range so the
    // loop keeps its 50 Hz cadence. A transaction that returns without an
    // I2C timeout proves the sensor is alive — even if its range_status
    // isn't Valid (e.g. target out of range) — so it refreshes the health
    // timestamp. Only a Valid measurement updates the cached range.
    if (sensor.dataReady()) {
        uint16_t mm = sensor.read(false);
        if (!sensor.timeoutOccurred()) {
            lastSampleMs = millis();
            everSampled = true;
            if (sensor.ranging_data.range_status == ::VL53L1X::RangeValid) {
                lastRange_m = mm / 1000.0f;
            }
        }
    }

    // Independent sensor-health flag for the status display: true iff the ToF
    // has answered its own I2C bus recently. Decoupled from the IMU and from
    // range quality, and self-clearing once the sensor stops responding —
    // mirroring the live read() health checks the BMP/MPU drivers expose.
    r.sensor_ok = everSampled && (millis() - lastSampleMs) < TOF_STALE_MS;

    // Subtract the boot baseline so range_m is the slant distance RISEN from
    // launch (~0 at rest). baselineRange_m is 0 until captureBaseline() runs.
    float relRange_m = isnan(lastRange_m) ? NAN : lastRange_m - baselineRange_m;
    r.range_m = relRange_m;

    // Tilt from the gravity vector: the body z-axis (which the ToF points
    // along, downward) is the rocket frame. When stationary, accel = -g in
    // the inertial frame, so its projection onto the body z-axis directly
    // gives g·cos(tilt). |accel_z| / |accel| is therefore cos(tilt) without
    // needing pitch/roll Euler angles or a square root in the denominator.
    //
    // Caveat: during powered ascent the accel vector includes thrust, so
    // this is most accurate at hover/near-hover. The fusion filter will
    // replace this with a gyro-integrated attitude estimate later.
    float a_mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (a_mag < 1.0f || isnan(lastRange_m)) {
        // Free-fall or no valid range yet — cannot trust the correction.
        r.vertical_m = NAN;
        r.tilt_rad = NAN;
        r.ok = false;
        return r;
    }

    float cos_tilt = fabsf(accel_z) / a_mag;
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;       // guard against FP rounding
    r.tilt_rad = acosf(cos_tilt);
    r.vertical_m = relRange_m * cos_tilt;
    r.ok = true;
    return r;
}
