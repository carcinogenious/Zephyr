// config.h — Zephyr EDF TVC Rocket
//
// SINGLE SOURCE OF TRUTH for every hardware-specific constant: pin map, I2C
// addresses, PID gains, deflection/throttle limits, ESC arming, telemetry, and
// safety cutoffs. Nothing magic should be buried in module logic — if it is a
// number tied to the airframe, the electronics, or tuning, it lives here.
//
// Values marked TUNE are tuned on the test stand; values marked CALIBRATE depend
// on the physical build (resistor divider, pushrod geometry) and must be measured.
//
// Board: Heltec WiFi LoRa 32 V3 (ESP32-S3FN8). See CLAUDE.md for pin cautions.

#pragma once

#include <cstdint>

namespace cfg {

// ───────────────────────────── I2C bus ──────────────────────────────────────
constexpr uint8_t PIN_I2C_SDA = 41;
constexpr uint8_t PIN_I2C_SCL = 42;
constexpr uint32_t I2C_CLOCK_HZ = 400000;  // 400 kHz fast mode

// Sensor I2C addresses
constexpr uint8_t ADDR_BMP388  = 0x76;  // baro (0x77 if SDO high)
constexpr uint8_t ADDR_MPU6050 = 0x68;  // IMU  (0x69 if AD0 high)
constexpr uint8_t ADDR_VL53L1X = 0x29;  // ToF
constexpr uint8_t ADDR_OLED    = 0x3C;  // on-board SSD1306 (Wire1, BSP pins)

// ──────────────────────────── Actuator pins ─────────────────────────────────
constexpr uint8_t PIN_ESC        = 47;  // EDF throttle, 1000–2000us ESC signal
constexpr uint8_t PIN_SERVO_PITCH = 48; // TVC pitch (MG90S)
constexpr uint8_t PIN_SERVO_YAW   = 3;  // TVC yaw   (MG90S)
constexpr uint8_t PIN_RECOVERY    = 26; // parachute deploy trigger (servo/MOSFET)

// ───────────────────────────── I/O pins ─────────────────────────────────────
constexpr uint8_t PIN_ARM_BUTTON = 2;   // arm/launch, INPUT_PULLUP (active low)
constexpr uint8_t PIN_BATT_ADC   = 1;   // 6S battery voltage divider
constexpr uint8_t PIN_BUZZER     = 19;
constexpr uint8_t PIN_LED        = 20;
// OLED uses the Heltec BSP macros (SDA_OLED/SCL_OLED/RST_OLED) — do not reassign.

// ────────────────────────── PWM / servo signal ──────────────────────────────
constexpr uint16_t SERVO_MIN_US    = 1000;
constexpr uint16_t SERVO_MAX_US    = 2000;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t SERVO_PWM_HZ    = 50;   // standard analog servo frame

// ───────────────────────────── ESC / throttle ───────────────────────────────
constexpr uint16_t ESC_MIN_US     = 1000;  // idle / disarmed (0% throttle)
constexpr uint16_t ESC_MAX_US     = 2000;  // 100% throttle
constexpr uint16_t ESC_ARM_US     = 1000;  // hold at min to arm
constexpr uint32_t ESC_ARM_MS     = 2000;  // hold duration before ESC arms (beeps)
constexpr float    THROTTLE_HOVER = 0.45f; // ~hover (T/W ~1.9), fraction 0..1
constexpr float    THROTTLE_LAUNCH = 0.65f; // TUNE — launch/ascent throttle
constexpr float    THROTTLE_MAX    = 0.90f; // safety cap below full-send
constexpr float    THROTTLE_RAMP_PER_S = 1.0f; // max throttle change /s (slew limit)

// ─────────────────────────── TVC (gimbaled nozzle) ──────────────────────────
constexpr float MAX_DEFLECTION_DEG = 12.0f;  // HARD clamp, both axes — never exceed
// Servo µs per degree of NOZZLE deflection. Pushrod/horn geometry dependent.
constexpr float TVC_US_PER_DEG = 11.0f;       // CALIBRATE from linkage geometry
// Per-axis mechanical zero trim (set when pushrods are adjusted, Build Step 5).
constexpr uint16_t TVC_PITCH_CENTER_US = SERVO_CENTER_US; // CALIBRATE
constexpr uint16_t TVC_YAW_CENTER_US   = SERVO_CENTER_US; // CALIBRATE
// Deflection sign per axis. VERIFY ON TEST STAND before any free flight:
// positive attitude error must produce a CORRECTING deflection, not runaway.
constexpr int8_t TVC_PITCH_SIGN = +1;  // VERIFY
constexpr int8_t TVC_YAW_SIGN   = +1;  // VERIFY

// ───────────────────────────── Control loop ─────────────────────────────────
constexpr uint16_t CONTROL_LOOP_HZ = 50;
constexpr float    CONTROL_DT      = 1.0f / CONTROL_LOOP_HZ;  // seconds

// PID gains — one controller per axis. All TUNE on the test stand.
constexpr float KP_PITCH = 0.0f, KI_PITCH = 0.0f, KD_PITCH = 0.0f;  // TUNE
constexpr float KP_YAW   = 0.0f, KI_YAW   = 0.0f, KD_YAW   = 0.0f;  // TUNE
// Integrator anti-windup: clamp the integral term's contribution, in degrees.
constexpr float PID_I_CLAMP_DEG = 6.0f;

// ────────────────────────── Sensor fusion ───────────────────────────────────
constexpr float FUSION_ALPHA   = 0.98f; // gyro/accel complementary (attitude)
constexpr float ALT_BARO_ALPHA = 0.98f; // baro vs accel-integrated altitude
constexpr float ALT_TOF_BETA   = 0.70f; // ToF weight near ground
constexpr float TOF_MAX_RANGE_M    = 4.0f;  // VL53L1X usable range
constexpr float TOF_TILT_LIMIT_DEG = 20.0f; // ignore ToF beyond this tilt

// ────────────────────────────── Battery (6S) ────────────────────────────────
constexpr uint8_t BATT_CELLS = 6;
// Divider scales 6S (25.2V max) into the 0–3.3V ADC range. ratio = (R1+R2)/R2.
constexpr float BATT_DIVIDER_RATIO = 8.0f;   // CALIBRATE against a multimeter
constexpr float ADC_VREF           = 3.3f;
constexpr uint16_t ADC_MAX_COUNTS  = 4095;   // 12-bit ESP32 ADC
constexpr float BATT_CELL_WARN_V = 3.50f;    // per-cell warning  (pack 21.0V)
constexpr float BATT_CELL_CRIT_V = 3.30f;    // per-cell cutoff   (pack 19.8V)
constexpr float BATT_WARN_V = BATT_CELLS * BATT_CELL_WARN_V;
constexpr float BATT_CRIT_V = BATT_CELLS * BATT_CELL_CRIT_V;

// ──────────────────────── Recovery / apogee detect ──────────────────────────
constexpr float    APOGEE_VEL_THRESH_MS = 0.0f;  // baro vertical velocity sign flip
constexpr uint16_t APOGEE_CONFIRM_MS    = 200;   // debounce before deploy
constexpr uint16_t RECOVERY_DEPLOY_US   = 2000;  // servo/trigger pulse to release

// ───────────────────────── Safety cutoffs (override) ────────────────────────
constexpr float    SAFETY_MAX_TILT_DEG = 30.0f;  // abort: cut throttle, deploy
constexpr float    SAFETY_MAX_ALT_M    = 30.0f;  // runaway altitude cap
constexpr uint32_t SAFETY_SENSOR_TIMEOUT_MS = 200; // I2C stall -> abort

// ───────────────────────────── Telemetry (WiFi) ─────────────────────────────
constexpr char     WIFI_AP_SSID[]   = "Zephyr-Telemetry";
constexpr char     WIFI_AP_PASS[]   = "zephyr-flight";  // 8+ chars for WPA2
constexpr uint16_t TELEM_UDP_PORT   = 4210;
constexpr uint16_t TELEM_HZ         = 50;
constexpr char     TELEM_BROADCAST[] = "255.255.255.255";

// ──────────────────────────────── OLED ──────────────────────────────────────
constexpr uint8_t OLED_WIDTH  = 128;
constexpr uint8_t OLED_HEIGHT = 64;

}  // namespace cfg
