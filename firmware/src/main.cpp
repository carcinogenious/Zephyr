// main.cpp — Zephyr EDF TVC Rocket — ATTITUDE + WiFi (manual throttle) BRING-UP
// Attitude PID → 2 TVC servos (hold vertical), plus the WiFi control/telemetry
// page: live PID-gain tuning and a manual throttle slider. NO flight sequencer,
// NO altitude loop, NO battery/safety modules yet — those come later.
//
// Throttle starts at 0 and only goes live through the page: ARM runs the ESC
// arming handshake, LAUNCH makes the slider live, ABORT cuts + stands down. The
// slider can never spin a disarmed fan (throttle::setManual is arm-gated).
//
// ⚠ This build has NO battery cutoff and NO loop watchdog while the fan can spin.
//   Keep the FAN CLAMPED, keep bench runs short, and watch the pack voltage.
//
// Bench check (servos): tilt the board, the nozzle should deflect to OPPOSE the
// tilt. Wrong way → flip that axis's sign in config (TVC_PITCH_DIR/TVC_ROLL_DIR).
//
// All angles in DEGREES. Expected I2C scan: 0x68 (MPU6050).

#include <Arduino.h>
#include <Wire.h>

#include <zephyr_config.h>
#include <mpu6050.h>
#include <attitude.h>
#include <pid.h>
#include <tvc.h>
#include <throttle.h>
#include <wifi_link.h>
#include <display.h>

static bool mpuOk = false;

// One attitude PID per tilt axis (pitch, roll). Constructed with the config
// defaults; the WiFi page can retune the gains live (applied each loop below).
// Derivative is taken on the gyro rate; output clamped to ±cfg::TVC_CLAMP_DEG.
static pid::Controller pidPitch({cfg::KP_PITCH, cfg::KI_PITCH, cfg::KD_PITCH},
                                cfg::PID_I_CLAMP_DEG);
static pid::Controller pidRoll({cfg::KP_ROLL, cfg::KI_ROLL, cfg::KD_ROLL},
                               cfg::PID_I_CLAMP_DEG);

// Manual-throttle gate (no flight sequencer in this build): mirrors the real
// sequencer's arm→launch step so the slider is only live after a deliberate
// LAUNCH. The physical power switch is the master arm; the web ARM still runs the
// ESC arming handshake (throttle::arm() holds min so the ESC accepts throttle).
static bool launched = false;   // slider live only after LAUNCH (page shows FLYING)

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
    if (found == 0) Serial.println(F("  NONE — check SDA/SCL, 3.3V, common ground"));
}

void setup() {
    Serial.begin(115200);
    delay(300);

    display::init();
    display::message("Zephyr", "attitude + wifi");

    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_CLOCK_HZ);

    Serial.println(F("\nZephyr — attitude + WiFi (manual throttle) bring-up"));
    i2cScan();

    // Hold the board still: the MPU captures its gyro bias during init().
    mpuOk = mpu6050::init();
    Serial.print(F("MPU6050 : "));
    Serial.println(mpuOk ? F("OK") : F("FAIL"));

    // ESC FIRST → minimum (LEDC ch0): a safe idle, fan off. Then servos → centers
    // (ch1/ch2). The fan never spins until ARM + LAUNCH via the page.
    throttle::init();
    tvc::init();

    // WiFi AP + control/telemetry page (own task, core 0). Seed the page's PID
    // gains with the config defaults so the editor starts from the real values.
    wifi_link::init();
    wifi_link::seedGains({cfg::KP_PITCH, cfg::KI_PITCH, cfg::KD_PITCH},
                         {cfg::KP_ROLL, cfg::KI_ROLL, cfg::KD_ROLL});

    // Echo the EFFECTIVE compiled constants so the monitor confirms what's running
    // (a wrong gain/sign/clamp here is the first thing to rule out).
    Serial.println(F("\n--- compiled config ---"));
    Serial.printf("PITCH  Kp=%.3f Ki=%.3f Kd=%.3f\n", cfg::KP_PITCH, cfg::KI_PITCH, cfg::KD_PITCH);
    Serial.printf("ROLL   Kp=%.3f Ki=%.3f Kd=%.3f\n", cfg::KP_ROLL, cfg::KI_ROLL, cfg::KD_ROLL);
    Serial.printf("TVC    DIR pitch=%d roll=%d | clamp=%.1f deg | %.0f us/deg\n",
                  cfg::TVC_PITCH_DIR, cfg::TVC_ROLL_DIR, cfg::TVC_CLAMP_DEG, cfg::TVC_PITCH_US_PER_DEG_POS);
    Serial.printf("CTR    pitchCenter=%u roll Center=%u us | I-clamp=%.1f deg\n",
                  cfg::TVC_PITCH_CENTER_US, cfg::TVC_ROLL_CENTER_US, cfg::PID_I_CLAMP_DEG);
    Serial.printf("FUSE   alpha=%.3f | loop=%u Hz\n", cfg::FUSION_ALPHA, cfg::CONTROL_LOOP_HZ);
    Serial.println(F("Tilt the board SLOWLY, one axis at a time, and watch:"));
    Serial.println(F("  ax/ay/az  -> which axis sees gravity (~9.8); sanity of the IMU mount"));
    Serial.println(F("  accX==fusX at rest, and both track the SAME way through a tilt"));
    Serial.println(F("  P-term sign opposes the tilt; D-term opposes the motion"));
    Serial.println(F("--- live ---"));
}

void loop() {
    static const unsigned long LOOP_MS = 1000 / cfg::CONTROL_LOOP_HZ;  // 100 Hz → 10 ms
    static unsigned long lastLoop = 0;
    unsigned long now = millis();
    if (now - lastLoop < LOOP_MS) return;
    float dt_s = (now - lastLoop) / 1000.0f;
    lastLoop = now;

    mpu6050::Reading imu{};               // imu.ok = false until a good read
    if (mpuOk) imu = mpu6050::read();

    // Complementary filter → fused pitch/roll; latch the per-axis gyro RATES (deg/s)
    // for the PID derivative on a fresh sample, else hold the last estimate.
    static fusion::Attitude att{NAN, NAN};
    static bool attReady = false;
    static float pitchRate_dps = 0.0f, rollRate_dps = 0.0f;
    if (imu.ok) {
        att = fusion::update(imu.accel_x, imu.accel_y, imu.accel_z,
                             imu.gyro_x, imu.gyro_y, dt_s);
        pitchRate_dps = imu.gyro_x * RAD_TO_DEG;   // fusion pairs gx→pitch (Zephyr mount)
        rollRate_dps  = imu.gyro_y * RAD_TO_DEG;   //              gy→roll
        attReady = true;
    }

    // Apply the latest live gains from the page (seeded with the config defaults).
    // setGains only swaps coefficients — integral/derivative history is preserved,
    // so retuning mid-test is smooth.
    wifi_link::PidGains gp = wifi_link::pitchGains();
    wifi_link::PidGains gr = wifi_link::rollGains();
    pidPitch.setGains({gp.kp, gp.ki, gp.kd});
    pidRoll.setGains({gr.kp, gr.ki, gr.kd});

    // Attitude PID (setpoint 0° = vertical) → nozzle deflection → servos.
    float defPitch = NAN, defRoll = NAN;
    if (attReady) {
        defPitch = pidPitch.update(0.0f, att.pitch_deg, pitchRate_dps, dt_s, cfg::TVC_CLAMP_DEG);
        defRoll  = pidRoll.update (0.0f, att.roll_deg,  rollRate_dps,  dt_s, cfg::TVC_CLAMP_DEG);
        tvc::setDeflection(defPitch, defRoll);
    }

    // ── Manual-throttle gate (arm → launch → slider) ────────────────────────
    // Read all three edge commands first (so one loop can't swallow another), then
    // act. ABORT always wins.
    bool wantArm    = wifi_link::takeArm();
    bool wantLaunch = wifi_link::takeLaunch();
    bool wantAbort  = wifi_link::takeAbort();
    if (wantAbort) {
        throttle::disarm();
        launched = false;
        Serial.println(F("ABORT — throttle cut, idle"));
    } else if (wantArm && !throttle::armed()) {
        Serial.println(F("ARMING ESC — FAN MUST BE CLAMPED"));
        throttle::arm();             // blocking ~2 s (ESC arming handshake)
        launched = false;
        Serial.println(F("ARMED — LAUNCH to make the slider live"));
    } else if (wantLaunch && throttle::armed()) {
        launched = true;
        Serial.println(F("LAUNCH — manual throttle live, TVC active"));
    }

    // Slider is live only after LAUNCH; otherwise command 0. setManual still holds
    // the ESC at minimum until armed (defense in depth).
    float thrCmd = launched ? wifi_link::manualThrottle() : 0.0f;
    throttle::setManual(thrCmd, dt_s);

    // ── Telemetry snapshot for the web task (fire-and-forget) ───────────────
    uint8_t flightMode = throttle::armed() ? (launched ? 3 : 1) : 0;  // DISARMED/ARMED/FLYING
    wifi_link::Telemetry tlm{};
    tlm.vertical_in  = NAN;          // no ToF in this build
    tlm.setpoint_in  = NAN;
    tlm.pitch_deg    = attReady ? att.pitch_deg : NAN;
    tlm.roll_deg     = attReady ? att.roll_deg  : NAN;
    tlm.defPitch_deg = defPitch;
    tlm.defRoll_deg  = defRoll;
    tlm.throttlePct  = throttle::fraction() * 100.0f;
    tlm.battV        = 0.0f;         // no battery sense in this build
    tlm.phase        = (uint8_t)throttle::phase();
    tlm.flightMode   = flightMode;
    tlm.bmpOk = false; tlm.mpuOk = mpuOk; tlm.tofOk = false;
    wifi_link::publish(tlm);

    // Attitude DIAGNOSTIC (~5 Hz, two lines). Tilt SLOWLY, one axis at a time:
    //   line 1 = raw IMU + timing : is the loop actually getting fresh data, at
    //            100 Hz, with gravity on the expected axis and gyro signs sane?
    //   line 2 = estimate + control: accel truth vs fused (must agree in sign), and
    //            the PID P/D breakdown that produced each nozzle command.
    static uint8_t dbg = 0;
    if (++dbg >= 20) {
        dbg = 0;
        fusion::Attitude acc{NAN, NAN};
        if (imu.ok) acc = fusion::accelOnly(imu.accel_x, imu.accel_y, imu.accel_z);

        Serial.printf("IMU %s ax=%6.2f ay=%6.2f az=%6.2f m/s2 | gx=%6.1f gy=%6.1f gz=%6.1f dps | dt=%4.1fms (%3.0fHz)\n",
                      imu.ok ? "OK" : "--", imu.accel_x, imu.accel_y, imu.accel_z,
                      imu.gyro_x * RAD_TO_DEG, imu.gyro_y * RAD_TO_DEG, imu.gyro_z * RAD_TO_DEG,
                      dt_s * 1000.0f, dt_s > 0 ? 1.0f / dt_s : 0.0f);
        Serial.printf("EST accP=%6.1f accR=%6.1f | fusP=%6.1f fusR=%6.1f || "
                      "PIT P=%5.2f D=%5.2f out=%5.2f nz=%4uus | ROL P=%5.2f D=%5.2f out=%5.2f nz=%4uus\n",
                      acc.pitch_deg, acc.roll_deg,
                      attReady ? att.pitch_deg : NAN, attReady ? att.roll_deg : NAN,
                      pidPitch.lastP(), pidPitch.lastD(), defPitch, tvc::pitchMicros(),
                      pidRoll.lastP(),  pidRoll.lastD(),  defRoll,  tvc::rollMicros());
    }

    // OLED (~5 Hz): fused angles + servo pulses + throttle.
    static uint8_t oled = 0;
    if (++oled >= 20) {
        oled = 0;
        String l1 = String("P") + String(attReady ? att.pitch_deg : 0.0f, 1) +
                    " R" + String(attReady ? att.roll_deg : 0.0f, 1) + " deg";
        String l2 = String("nz ") + tvc::pitchMicros() + "/" + tvc::rollMicros() +
                    " t" + String(throttle::fraction() * 100.0f, 0);
        display::message(l1, l2);
    }
}
