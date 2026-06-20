// main.cpp — Zephyr EDF TVC Rocket — PHASE 1 FLIGHT FIRMWARE
// Attitude PID (2 TVC servos) + web manual-throttle (open-loop, voltage-compensated).
// All lengths in INCHES, all angles in DEGREES.
//
// Two INDEPENDENT controllers share this loop but write DIFFERENT actuators:
//   1. ATTITUDE PID  → the 2 TVC servos (hold vertical). Live from boot; with the
//      nozzle clamp it self-limits, so it's safe on the bench — tilt the stand and
//      the nozzle should deflect to OPPOSE the tilt (the per-axis SIGN check).
//   2. THROTTLE      → the ESC, from the web slider (Phase 1): manual fraction →
//      voltage-compensate → battery protection → ESC. Gated behind arm + launch;
//      the fan never spins on boot. Closed-loop altitude is Phase 2 (deferred).
// Voltage affects ONLY the throttle path; the servos don't care about pack voltage.
//
// Loop runs at cfg::CONTROL_LOOP_HZ (100 Hz) on core 1; WiFi runs on core 0 and can
// never gate it, so the slider can't make the attitude loop laggy. Baro is read at
// ~20 Hz (display only, NOT used for altitude) so it can't stall the fast loop.
//
// Expected I2C scan: 0x76 or 0x77 (BMP388), 0x68 (MPU6050), 0x29 (VL53L1X).

#include <Arduino.h>
#include <Wire.h>

#include <zephyr_config.h>
#include <bmp388.h>
#include <mpu6050.h>
#include <tof.h>
#include <attitude.h>
#include <pid.h>
#include <tvc.h>
#include <throttle.h>
#include <wifi_link.h>
#include <display.h>
#include <state_machine.h>
#include <safety.h>
#include <battery.h>

static const float M_TO_IN = 39.3701f;   // meters → inches for the readout

static bool bmpOk = false;
static bool mpuOk = false;
static bool tofOk = false;

// One attitude PID per controlled tilt axis (pitch, roll). Gains + integral clamp
// come from config; the derivative is taken on the gyro rate (passed per tick) and
// the output is clamped to ±cfg::TVC_CLAMP_DEG. See the data-flow note in loop().
static pid::Controller pidPitch({cfg::KP_PITCH, cfg::KI_PITCH, cfg::KD_PITCH},
                                cfg::PID_I_CLAMP_DEG);
static pid::Controller pidRoll({cfg::KP_ROLL, cfg::KI_ROLL, cfg::KD_ROLL},
                              cfg::PID_I_CLAMP_DEG);

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

static const char* phaseName(throttle::Phase p) {
    switch (p) {
        case throttle::Phase::MANUAL:  return "MANUAL";
        case throttle::Phase::ASCEND:  return "ASCEND";
        case throttle::Phase::HOVER:   return "HOVER";
        case throttle::Phase::DESCEND: return "DESCEND";
        case throttle::Phase::LANDED:  return "LANDED";
        default:                       return "IDLE";
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);

    // OLED first so init progress is visible on the panel. display::init()
    // powers the Vext rail (GPIO 36, active-LOW) and pulses RST_OLED — the
    // fix for the "OLED won't turn on" issue carried over from SPARC.
    display::init();
    display::message("Zephyr", "phase 1");

    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_CLOCK_HZ);

    Serial.println(F("\nZephyr — Phase 1 (attitude PID + manual throttle)"));
    Serial.println(F("Heltec WiFi LoRa 32 V3 (ESP32-S3)"));
    i2cScan();

    // Hold the board still through boot: the BMP/ToF capture their launch-zero
    // baselines (so both read ~0 at the pad — relative height) and the MPU
    // captures gyro bias here.
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

    // Battery sense (lib/battery): take one reading so the pad screen shows a
    // real pack voltage immediately.
    battery::init();
    battery::update();
    display::sensorStatus(bmpOk, mpuOk, tofOk, battery::volts(), battery::cellV(),
                          battery::percent(), battery::isLow());

    // TVC servos: attach (native LEDC ch1/ch2) and center the nozzle. Active
    // continuously from boot, but the nozzle only deflects under PID command.
    tvc::init();

    // ESC starts disarmed (fan off). It only spins after a deliberate arm +
    // launch via the web page — never on boot. FAN MUST BE CLAMPED.
    throttle::init();

    // Flight sequencer (lib/state_machine): resets to DISARMED. Owns ESC arming,
    // launch gating, and the landing summary. Arm/launch/abort come from the web.
    state_machine::init();

    // Loop watchdog (lib/safety): a monitor task on core 0 cuts the ESC if this
    // loop ever stalls. Started after throttle::init() so the ESC exists to cut.
    safety::init();

    // WiFi AP + control/telemetry page. Convenience layer only — runs in its own
    // task and can never gate the control loop. Prints the AP name + URL.
    wifi_link::init();

    Serial.println(F("ESC disarmed. ARM + LAUNCH via web, then the throttle slider (FAN CLAMPED)."));
    Serial.println(F("--- live readings (inches / degrees) ---"));
}

void loop() {
    // Fixed control rate (cfg::CONTROL_LOOP_HZ): tight dt for the complementary
    // filter + attitude PID; the serial/OLED refresh is decimated below.
    static const unsigned long LOOP_MS = 1000 / cfg::CONTROL_LOOP_HZ;
    static unsigned long lastLoop = 0;
    unsigned long now = millis();
    if (now - lastLoop < LOOP_MS) return;
    float dt_s = (now - lastLoop) / 1000.0f;
    lastLoop = now;

    // Barometer (display only — NOT used for altitude): read at ~20 Hz so its
    // slow conversion can't stall the fast attitude loop. Holds the last value.
    static float altBaro = NAN;
    static uint8_t baroDiv = 0;
    if (bmpOk && ++baroDiv >= (cfg::CONTROL_LOOP_HZ / 20)) {
        baroDiv = 0;
        altBaro = bmp388::readAltitude();           // meters AGL (relative to pad ~0)
    }

    mpu6050::Reading imu{};                          // imu.ok = false
    if (mpuOk) imu = mpu6050::read();

    tof::Reading rng{};
    rng.range_m = NAN;
    if (tofOk) {
        rng = imu.ok ? tof::read(imu.accel_x, imu.accel_y, imu.accel_z)
                     : tof::read(0.0f, 0.0f, 0.0f);
    }

    // Attitude estimate (complementary filter): gyro for the fast, thrust-immune
    // short term, accel as the gravity reference. Step it on a fresh IMU sample and
    // latch the per-axis gyro RATES (deg/s) for the PID derivative; otherwise hold.
    static fusion::Attitude att{NAN, NAN};
    static bool attReady = false;
    static float pitchRate_dps = 0.0f, rollRate_dps = 0.0f;
    if (imu.ok) {
        att = fusion::update(imu.accel_x, imu.accel_y, imu.accel_z,
                             imu.gyro_x, imu.gyro_y, dt_s);
        pitchRate_dps = imu.gyro_x * RAD_TO_DEG;     // d(pitch)/dt  (Zephyr mount: gx→pitch)
        rollRate_dps  = imu.gyro_y * RAD_TO_DEG;     // d(roll)/dt   (gy→roll)
        attReady = true;
    }

    // ── Attitude → TVC PID (control data flow: which value feeds what) ───────
    // MPU6050 accel+gyro ─▶ fusion ─▶ pitch_deg, roll_deg in the GIMBAL frame
    //   (Zephyr's MPU mount is resolved inside lib/fusion, so each axis is clean).
    //   pitch_deg (+ gyro rate) ─▶ pitch PID, setpoint 0° ─▶ pitch servo
    //   roll_deg  (+ gyro rate) ─▶ roll   PID, setpoint 0° ─▶ roll   servo
    //   ─▶ lib/tvc maps degrees ─▶ servo µs (center + 30 µs/° + per-axis SIGN) and
    //   applies the authoritative ±cfg::TVC_CLAMP_DEG clamp, then writes the servos.
    // The derivative is the gyro rate (clean, no setpoint kick); the PID also clamps
    // its output to ±TVC_CLAMP_DEG and freezes the integral when saturated there.
    float defPitch = NAN;   // PID-commanded pitch-axis deflection, deg (pre-tvc-clamp)
    float defRoll   = NAN;   // PID-commanded roll-axis   deflection, deg (pre-tvc-clamp)
    if (attReady) {
        defPitch = pidPitch.update(0.0f, att.pitch_deg, pitchRate_dps, dt_s, cfg::TVC_CLAMP_DEG);
        defRoll  = pidRoll.update (0.0f, att.roll_deg,  rollRate_dps,  dt_s, cfg::TVC_CLAMP_DEG);
        tvc::setDeflection(defPitch, defRoll);        // re-clamps + writes the servos
    }

    // ToF slant range tilt-corrected to true vertical with the FUSED attitude
    // (cos(pitch)·cos(roll)) — height above the pad (~0 at rest). Telemetry/display
    // only in Phase 1; it becomes the altitude controller's feedback in Phase 2.
    float vertical_m = NAN;
    if (attReady && !isnan(rng.range_m)) {
        vertical_m = rng.range_m * cosf(att.pitch_deg * DEG_TO_RAD)
                                 * cosf(att.roll_deg * DEG_TO_RAD);
    }

    // Battery: sample + classify every loop (kept up in flight too). The sequencer
    // reads the latched state to ramp down / cut on a low pack.
    battery::update();

    // ── Flight sequencer: DISARMED→ARMED→COUNTDOWN→FLYING ────────────────────
    // Feed this loop's state and the web slider value; the sequencer owns the
    // arm/launch/abort events, ESC arming, the manual-throttle hand-off, battery +
    // safety protection, and the landing summary.
    state_machine::Inputs smIn{};
    smIn.vertical_m     = vertical_m;
    smIn.pitch_deg      = attReady ? att.pitch_deg : NAN;
    smIn.roll_deg       = attReady ? att.roll_deg : NAN;
    smIn.attReady       = attReady;
    smIn.mpuOk          = mpuOk;
    smIn.tofOk          = tofOk;
    smIn.imuLive        = imu.ok;                    // fresh sample this loop (failsafe)
    smIn.tofLive        = rng.sensor_ok;             // ToF answered this loop
    smIn.dt_s           = dt_s;
    smIn.manualThrottle = wifi_link::manualThrottle();  // web slider, 0..1 (Phase 1)
    state_machine::Mode flightMode = state_machine::update(smIn);

    // Publish the telemetry snapshot for the web task (fire-and-forget copy).
    wifi_link::Telemetry tlm{};
    tlm.vertical_in   = isnan(vertical_m) ? NAN : vertical_m * M_TO_IN;
    tlm.setpoint_in   = NAN;                         // no altitude setpoint in Phase 1
    tlm.pitch_deg     = attReady ? att.pitch_deg : NAN;
    tlm.roll_deg      = attReady ? att.roll_deg : NAN;
    tlm.defPitch_deg  = defPitch;
    tlm.defRoll_deg    = defRoll;
    tlm.throttlePct   = throttle::fraction() * 100.0f;   // ACTUAL (compensated) %
    tlm.battV         = battery::volts();
    tlm.phase         = (uint8_t)throttle::phase();
    tlm.flightMode    = (uint8_t)flightMode;
    tlm.bmpOk = bmpOk; tlm.mpuOk = mpuOk; tlm.tofOk = tofOk;
    wifi_link::publish(tlm);

    // ── Live serial readout (~5 Hz) ────────────────────────────────────────
    static uint8_t dbgCount = 0;
    if (++dbgCount >= 20) {
        dbgCount = 0;

        float pitch  = attReady ? att.pitch_deg : NAN;
        float roll   = attReady ? att.roll_deg  : NAN;
        float baroIn = isnan(altBaro)     ? NAN : altBaro     * M_TO_IN;  // baro (display)
        float tofIn  = isnan(rng.range_m) ? NAN : rng.range_m * M_TO_IN;  // slant range
        float vertIn = isnan(vertical_m)  ? NAN : vertical_m  * M_TO_IN;  // ToF vertical

        Serial.print(F("baro="));      Serial.print(fmt(baroIn, "in"));
        Serial.print(F("  pitch="));   Serial.print(fmt(pitch, "deg"));
        Serial.print(F("  roll="));    Serial.print(fmt(roll, "deg"));
        Serial.print(F("  vert="));    Serial.print(fmt(vertIn, "in"));
        Serial.print(F("  || defP=")); Serial.print(fmt(defPitch, "deg"));
        Serial.print(F("  defY="));    Serial.print(fmt(defRoll, "deg"));
        Serial.print(F("  || thr="));  Serial.print(throttle::fraction() * 100.0f, 0);
        Serial.print(F("% "));         Serial.print(phaseName(throttle::phase()));
        Serial.print(F("  || batt=")); Serial.print(battery::volts(), 2);
        Serial.print(F("V "));         Serial.print(battery::percent(), 0);
        Serial.print(F("%"));          if (battery::isLow()) Serial.print(F(" LOW"));
        Serial.println();
    }

    // OLED pad readout (~2 Hz) — sensor health + battery. Only on the pad
    // (DISARMED): in flight the sequencer owns the panel.
    static uint8_t oledCount = 0;
    if (flightMode == state_machine::Mode::DISARMED && ++oledCount >= 50) {
        oledCount = 0;
        display::sensorStatus(/* bmp */ !isnan(altBaro),
                              /* mpu */ imu.ok,
                              /* tof */ rng.sensor_ok,
                              battery::volts(), battery::cellV(),
                              battery::percent(), battery::isLow());
    }

    // Pet the loop watchdog last: reaching here means a full control iteration
    // completed. If the loop ever hangs before this, lib/safety cuts the ESC.
    safety::feed();
}
