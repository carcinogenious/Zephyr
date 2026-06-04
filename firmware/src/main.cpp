// main.cpp — Zephyr EDF TVC Rocket
// SENSOR BRING-UP build. I2C scan, per-sensor init, OLED status, and a live
// readout: barometric altitude, fused pitch/roll, ToF slant range, and the
// trig-corrected true vertical height. All lengths in INCHES, all angles in
// DEGREES. This is what you flash to verify the wired breadboard before any
// flight logic exists.
//
// Modules reused from SPARC: sensors (firmware/lib/sensors/), fusion
// (firmware/lib/fusion/), display (firmware/lib/display/). PID, TVC, throttle,
// WiFi telemetry, and the state machine land in their lib/ modules later.
//
// Expected I2C scan: 0x76 or 0x77 (BMP388), 0x68 (MPU6050), 0x29 (VL53L1X).

#include <Arduino.h>
#include <Wire.h>

#include <zephyr_config.h>
#include <bmp388.h>
#include <mpu6050.h>
#include <tof.h>
#include <attitude.h>
#include <display.h>

static const float M_TO_IN = 39.3701f;   // meters → inches for the readout

static bool bmpOk = false;
static bool mpuOk = false;
static bool tofOk = false;

// Sweep the I2C bus and print every device that ACKs. A blank result almost
// always means a wiring/ground problem (see docs/wiring-diagram.md).
static void i2cScan() {
    Serial.println(F("I2C scan:"));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  found 0x"));
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) {
        Serial.println(F("  NONE — check SDA/SCL, 3.3V, and common ground"));
    }
}

// One readout field: "<value><unit>" with two decimals, or "FAIL" when the
// value is unavailable (NAN), so a missing sensor never prints a misleading 0.
static String fmt(float v, const char* unit) {
    return isnan(v) ? String("FAIL") : String(v, 2) + unit;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    // OLED first so init progress is visible on the panel. display::init()
    // powers the Vext rail (GPIO 36, active-LOW) and pulses RST_OLED — the
    // fix for the "OLED won't turn on" issue carried over from SPARC.
    display::init();
    display::message("Zephyr", "sensor test");

    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_CLOCK_HZ);

    Serial.println(F("\nZephyr — sensor bring-up"));
    Serial.println(F("Heltec WiFi LoRa 32 V3 (ESP32-S3)"));
    i2cScan();

    // Hold the board still through boot: the BMP/ToF capture their launch-zero
    // baselines and the MPU captures gyro bias here.
    bmpOk = bmp388::init();
    Serial.print(F("BMP388  : "));
    Serial.println(bmpOk ? F("OK") : F("FAIL"));
    if (bmpOk) {
        display::message("Waiting for", "pressure...");
        Serial.print(F("  baseline: "));
        Serial.println(bmp388::captureBaseline(10000) ? F("stable")
                                                      : F("UNSTABLE (timeout)"));
    }

    mpuOk = mpu6050::init();
    Serial.print(F("MPU6050 : "));
    Serial.println(mpuOk ? F("OK") : F("FAIL"));

    tofOk = tof::init();
    Serial.print(F("VL53L1X : "));
    Serial.println(tofOk ? F("OK") : F("FAIL"));
    if (tofOk) {
        display::message("Waiting for", "ToF zero...");
        Serial.print(F("  baseline: "));
        Serial.println(tof::captureBaseline(10000) ? F("stable")
                                                   : F("UNSTABLE (timeout)"));
    }

    // Battery sense not wired in the bring-up build — show 0.00 V for now.
    display::sensorStatus(bmpOk, mpuOk, tofOk, /* battV */ 0.0f);
    Serial.println(F("--- live readings (inches / degrees, ~5 Hz) ---"));
}

void loop() {
    // 50 Hz cadence so the complementary filter integrates the gyro on a tight
    // dt; the serial/OLED refresh is decimated below.
    static unsigned long lastLoop = 0;
    unsigned long now = millis();
    if (now - lastLoop < 20) return;
    float dt_s = (now - lastLoop) / 1000.0f;
    lastLoop = now;

    // Only poll a sensor that answered at boot — polling a missing one would
    // wait out its I2C timeout and stall the shared bus for the others.
    float altBaro = bmpOk ? bmp388::readAltitude() : NAN;   // meters AGL

    mpu6050::Reading imu{};                                  // imu.ok = false
    if (mpuOk) imu = mpu6050::read();

    tof::Reading rng{};
    rng.range_m = NAN;
    if (tofOk) {
        rng = imu.ok ? tof::read(imu.accel_x, imu.accel_y, imu.accel_z)
                     : tof::read(0.0f, 0.0f, 0.0f);
    }

    // Attitude estimate (complementary filter): gyro for the fast, thrust-
    // immune short term, accel as the gravity reference. Step it only on a
    // fresh IMU sample; otherwise hold the last angles.
    static fusion::Attitude att{NAN, NAN};
    static bool attReady = false;
    if (imu.ok) {
        att = fusion::update(imu.accel_x, imu.accel_y, imu.accel_z,
                             imu.gyro_x, imu.gyro_y, dt_s);
        attReady = true;
    }

    // ── Live serial readout (~5 Hz) ────────────────────────────────────────
    // Effective vertical height is a right triangle: the ToF measures the SLANT
    // range along the rocket's long axis (hypotenuse) and the IMU gives the
    // tilt off vertical (angle), so true height = range · cos(pitch) · cos(roll).
    // Printed next to the barometer's independent altitude plus their difference,
    // so disagreement between the two is visible at a glance.
    static uint8_t dbgCount = 0;
    if (++dbgCount >= 10) {
        dbgCount = 0;

        float pitch  = attReady ? att.pitch_deg : NAN;                   // deg, signed
        float roll   = attReady ? att.roll_deg  : NAN;                   // deg, signed
        float baroIn = isnan(altBaro)     ? NAN : altBaro     * M_TO_IN;  // baro altitude
        float tofIn  = isnan(rng.range_m) ? NAN : rng.range_m * M_TO_IN;  // slant range
        float cosTilt = attReady ? cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD)
                                 : NAN;
        float vertIn = (isnan(cosTilt) || isnan(tofIn)) ? NAN : tofIn * cosTilt;
        float dIn    = (isnan(vertIn) || isnan(baroIn)) ? NAN : baroIn - vertIn;

        Serial.print(F("baro="));        Serial.print(fmt(baroIn, "in"));
        Serial.print(F("  pitch="));     Serial.print(fmt(pitch, "deg"));
        Serial.print(F("  roll="));      Serial.print(fmt(roll, "deg"));
        Serial.print(F("  tof="));       Serial.print(fmt(tofIn, "in"));
        Serial.print(F("  | vert="));    Serial.print(fmt(vertIn, "in"));
        Serial.print(F("  baro-vert=")); Serial.print(fmt(dIn, "in"));
        Serial.println();
    }

    // OLED sensor status refresh (~2 Hz). Each row is a live health check:
    // BMP/MPU report their own read() success; rng.sensor_ok is the ToF's own
    // I2C liveness (kept separate from rng.ok, which needs the IMU).
    static uint8_t oledCount = 0;
    if (++oledCount >= 25) {
        oledCount = 0;
        display::sensorStatus(/* bmp */ !isnan(altBaro),
                              /* mpu */ imu.ok,
                              /* tof */ rng.sensor_ok,
                              /* batt */ 0.0f);
    }
}
