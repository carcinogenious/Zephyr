#include "bmp388.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

namespace {
Adafruit_BMP3XX bmp;

// Standard sea-level reference for the barometric formula. readAltitude()
// produces an absolute pressure-altitude against this; the launch-relative
// (AGL) zero is the baseline captured at boot by captureBaseline().
const float SEA_LEVEL_HPA = 1013.25f;

// Launch-pad altitude captured at boot, subtracted from every reading so the
// reported value is altitude RELATIVE TO LAUNCH (AGL). Re-zeroes on each
// power-up, so it stays accurate regardless of weather or location — no
// hand-edited constant to keep in sync.
float baselineAlt_m = 0.0f;

// Optional manual trim added on top of the captured zero, in INCHES (matches
// the serial readout). Leave at 0 for a true launch-relative zero; set it
// only to deliberately offset the reported altitude.
const float BASELINE_TRIM_IN = 0.0f;

// Drift tolerance for the boot baseline capture, in INCHES. The reading is
// "settled" once two consecutive ~1 s window averages differ by less than
// this — i.e. warmup/ambient drift has died down. A single quiet window isn't
// enough: a slowly drifting baro looks quiet sample-to-sample yet keeps
// wandering, which is what let a one-shot capture lock a baseline tens of
// inches off its final value.
const float SETTLE_DRIFT_IN = 6.0f;

// Standard barometric formula — absolute pressure-altitude in meters.
float pressureToAltitude(float pressure_hPa) {
    return 44330.0f * (1.0f - powf(pressure_hPa / SEA_LEVEL_HPA, 0.1903f));
}
}  // namespace

bool bmp388::init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);

    // GY-BMP388 modules ship strapped to either 0x77 or 0x76 — try both.
    if (!bmp.begin_I2C(0x77, &Wire) && !bmp.begin_I2C(cfg::ADDR_BMP388, &Wire)) {
        return false;
    }

    // Drone-style config: heavy pressure oversampling plus the IIR filter
    // for low-noise altitude at the 50 Hz control-loop rate.
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Warm up the IIR filter and confirm the sensor actually returns data.
    // The launch zero is captured separately by captureBaseline() once the
    // reading settles. Fail init if nothing comes back.
    int samples = 0;
    for (int i = 0; i < 20; i++) {
        if (bmp.performReading()) {
            samples++;
        }
        delay(20);
    }
    return samples > 0;
}

bool bmp388::captureBaseline(unsigned long timeoutMs) {
    const int WINDOW = 10;                   // ~1 s of samples at 10 Hz
    const unsigned long PERIOD_MS = 100;
    const float settle_m = SETTLE_DRIFT_IN / 39.3701f;

    float prevMean = NAN;
    unsigned long start = millis();

    while (millis() - start < timeoutMs) {
        // Average one ~1 s window of pressure-altitude.
        float sum = 0.0f;
        int n = 0;
        for (int i = 0; i < WINDOW && (millis() - start < timeoutMs); i++) {
            if (bmp.performReading()) {
                sum += pressureToAltitude(bmp.pressure / 100.0f);
                n++;
            }
            delay(PERIOD_MS);
        }
        if (n == 0) continue;
        float mean = sum / n;

        // Lock the baseline only when consecutive window means barely move:
        // warmup/ambient drift has died down, so this mean won't wander away
        // after we zero to it. (A single quiet window can still be mid-drift.)
        if (!isnan(prevMean) && fabsf(mean - prevMean) < settle_m) {
            baselineAlt_m = mean;
            return true;
        }
        prevMean = mean;
    }

    // Timed out without settling — best-effort baseline from the last window
    // so altitude is at least launch-relative, and report the failure.
    if (!isnan(prevMean)) {
        baselineAlt_m = prevMean;
    }
    return false;
}

float bmp388::readAltitude() {
    if (!bmp.performReading()) {
        return NAN;
    }
    // Absolute pressure-altitude, offset to launch-relative (AGL) by the
    // baseline captured at boot, plus the optional manual trim (inches->m).
    float alt_m = pressureToAltitude(bmp.pressure / 100.0f);  // bmp.pressure is in Pa
    return (alt_m - baselineAlt_m) + BASELINE_TRIM_IN / 39.3701f;
}
