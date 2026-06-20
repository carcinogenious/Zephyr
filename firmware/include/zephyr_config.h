// zephyr_config.h — Zephyr EDF TVC Rocket
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
// ESC signal is buffered through a 74AHCT125N (3.3V→5V) so the ESC arms; the
// buffer is transparent — LEDC drives PIN_ESC normally. Add a 10k pulldown
// from GPIO6→GND so the buffer reads a clean min during boot float.
constexpr uint8_t PIN_ESC        = 6;   // EDF throttle, 1000–2000us (was 47: USB-UART noisy)
constexpr uint8_t PIN_SERVO_PITCH = 48; // TVC pitch (MG90S)
constexpr uint8_t PIN_SERVO_ROLL   = 5;  // TVC roll   (MG90S) — NOT 3 (S3 strapping pin)
// Do not use GPIO 4 for PWM (demonstrated wiring fault). ESC moved off 47 because
// continuous PWM there was noisy (USB-UART adjacent).
// GPIO 26 is free (was the parachute trigger). Landing is propulsive — no deploy
// actuator. Leave 26 as a spare; see CLAUDE.md pin cautions.

// ───────────────────────────── I/O pins ─────────────────────────────────────
constexpr uint8_t PIN_BATT_ADC   = 1;   // 6S battery voltage divider
// No arm button, buzzer, or status LED — arm/launch/abort and all status are
// handled over the WiFi control page (lib/wifi_link). GPIO 2/19/20 are free.
// OLED uses the Heltec BSP macros (SDA_OLED/SCL_OLED/RST_OLED) — do not reassign.

// ────────────────────────── PWM / servo signal ──────────────────────────────
constexpr uint16_t SERVO_MIN_US    = 1000;
constexpr uint16_t SERVO_MAX_US    = 2000;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t SERVO_PWM_HZ    = 50;   // standard analog servo frame

// Native LEDC PWM (ESC + BOTH servos). All three outputs run on native LEDC via
// ledcAttachChannel() at SERVO_PWM_HZ on EXPLICIT channels — the only arrangement
// that drives all three on this S3 + Arduino core 3.x without an MCPWM collision
// (ESP32Servo's 3-instance combo corrupted the ESC; see CLAUDE.md). Attach each
// channel ONCE in its module's init(). µs→duty: ((uint64_t)us << RES) / PERIOD_US.
constexpr uint8_t  PWM_RESOLUTION_BITS = 14;     // LEDC duty resolution
constexpr uint32_t PWM_PERIOD_US       = 20000;  // 50 Hz frame
constexpr uint8_t  LEDC_CH_ESC   = 0;            // ESC   on LEDC channel 0
constexpr uint8_t  LEDC_CH_PITCH = 1;            // pitch servo on LEDC channel 1
constexpr uint8_t  LEDC_CH_ROLL   = 2;            // roll   servo on LEDC channel 2

// ───────────────────────────── ESC / throttle ───────────────────────────────
constexpr uint16_t ESC_MIN_US     = 1000;  // idle / disarmed (0% throttle)
constexpr uint16_t ESC_MAX_US     = 2000;  // 100% throttle
constexpr uint16_t ESC_ARM_US     = 1000;  // hold at min to arm
constexpr uint32_t ESC_ARM_MS     = 2000;  // hold duration before ESC arms (beeps)
constexpr float    THROTTLE_HOVER = 0.45f; // ~hover (T/W ~1.9), fraction 0..1
constexpr float    THROTTLE_LAUNCH = 0.65f; // TUNE — launch/ascent throttle
constexpr float    THROTTLE_MAX    = 0.90f; // safety cap below full-send
constexpr float    THROTTLE_RAMP_PER_S = 1.0f; // max throttle change /s (slew limit)

// Voltage compensation (lib/throttle). Open-loop throttle has no altitude feedback
// to absorb thrust loss as the pack drains, and thrust ≈ throttle·V² (RPM ∝ V,
// thrust ∝ RPM²). So scale the throttle ABOVE idle by (V_REF/V_now)^EXP to hold
// thrust roughly constant. V_REF is the full pack the base profile is calibrated at;
// V_now is the LIVE under-load reading. Result is clamped to THROTTLE_MAX; if comp
// would demand more, the pack is depleted — battery protection (RAMP/FLOOR) takes
// over and ALWAYS overrides compensation.
constexpr float    THROTTLE_V_REF_V   = 25.2f; // full pack the base profile assumes
constexpr float    THROTTLE_VCOMP_EXP = 2.0f;  // thrust ∝ V²; tune 1.5–2.0 on the bench

// ──────────── Flight profile (canned: ascend → hover → descend) ──────────────
// FOR NOW: climb to ALT_TARGET, hold HOVER_MS, then descend slowly and cut.
// Heights/durations are expected to change; stability is the invariant.
constexpr float    ALT_TARGET_M    = 1.524f;  // 5 ft hover altitude
constexpr float    ASCENT_RATE_MS  = 0.50f;   // setpoint climb rate, m/s
constexpr float    DESCENT_RATE_MS = 0.30f;   // setpoint descent rate (slower), m/s
constexpr uint32_t HOVER_MS        = 30000;   // hold at target, 30 s
constexpr float    ALT_LANDED_M    = 0.10f;   // descent complete below this → cut

// Launch sequencing / safety timeouts.
constexpr uint32_t LAUNCH_COUNTDOWN_MS = 3000;   // re-verify sensors before spool
constexpr uint32_t FLIGHT_TIMEOUT_MS   = 45000;  // hard throttle cut if seq runs long

// ─────────────────────────── TVC (gimbaled nozzle) ──────────────────────────
// The two gimbal servos run on native LEDC (ledcAttachChannel, ch1/ch2), sharing
// the 50 Hz timer with the ESC (ch0) — the proven 3-PWM coexistence path on this
// board. lib/tvc drives them by commanding the target pulse and letting the servo
// slew natively (no software interpolation, write-on-change) for smooth vectoring.

// Deflection clamp (degrees) — PRIMARY limit, both axes. TUNING value: mechanical
// max is ±20°, kept well inside so untuned PID oscillation stays gentle. Raise
// toward ~18° for flight AFTER the PID is tuned. One knob, change in one place.
constexpr float TVC_CLAMP_DEG = 15.0f;
constexpr float MAX_DEFLECTION_DEG = TVC_CLAMP_DEG;  // legacy alias (lib/tvc, safety)

// Bench-measured per-axis center pulse (µs). The two axes differ — this is normal
// (nonsymmetric linkage); per-axis constants absorb it. Do NOT force both to 1500.
constexpr uint16_t TVC_PITCH_CENTER_US = 1600;  // CALIBRATE (bench)
constexpr uint16_t TVC_ROLL_CENTER_US   = 1700;  // CALIBRATE (bench)

// Per-axis, per-SIDE slope (µs per degree), bench-measured on the mounted gimbal.
// BOTH axes are LINEAR and SYMMETRIC at 30 µs/deg (same both sides). The per-side
// fields are retained so an axis CAN be made asymmetric, but both sides are 30 here.
//   pitch: ±20° → 1600 ± 600µs  (30/deg both sides)
//   roll:   ±20° → 1700 ± 600µs  (30/deg both sides)
constexpr float TVC_PITCH_US_PER_DEG_POS = 30.0f;
constexpr float TVC_PITCH_US_PER_DEG_NEG = 30.0f;
constexpr float TVC_ROLL_US_PER_DEG_POS   = 30.0f;
constexpr float TVC_ROLL_US_PER_DEG_NEG   = 30.0f;

// Per-axis direction sign (+1/-1). VERIFY ON STAND: positive attitude error must
// command a CORRECTING deflection (back toward vertical), not runaway. Do not
// assume — tilt the rocket on the stand and flip here if it drives the wrong way.
constexpr int8_t TVC_PITCH_DIR = -1;  // flipped: +1 drove the nozzle WITH the tilt (runaway); VERIFY each axis
constexpr int8_t TVC_ROLL_DIR   = -1;  // flipped: +1 drove the nozzle WITH the tilt (runaway); VERIFY each axis

// Final hard µs backstop per axis (mechanical-safe limits) — a bad value can never
// drive a servo into its bind. The degree clamp is primary; this is the backstop.
// Set ≈ the ±20° mechanical extremes (center ± 20°·slope); tighten to the exact
// bench-measured safe min/max once known. Wide enough not to clip a legit ≤18° cmd.
constexpr uint16_t TVC_PITCH_US_MIN = 1000;  // -20° mechanical (1600 - 20·30)
constexpr uint16_t TVC_PITCH_US_MAX = 2200;  // +20° mechanical (1600 + 20·30)
constexpr uint16_t TVC_ROLL_US_MIN   = 1100;  // -20° mechanical (1700 - 20·30)
constexpr uint16_t TVC_ROLL_US_MAX   = 2300;  // +20° mechanical (1700 + 20·30)

// ───────────────────────────── Control loop ─────────────────────────────────
// 100 Hz fixed rate: tight enough for the attitude loop to feel crisp (servos
// track the fused angle), comfortably within the I²C + servo/ESC write budget.
constexpr uint16_t CONTROL_LOOP_HZ = 100;
constexpr float    CONTROL_DT      = 1.0f / CONTROL_LOOP_HZ;  // seconds

// Attitude PID gains — one controller per tilt axis (pitch, roll), output in DEGREES
// of nozzle deflection. STARTING values, refined on the CoM-pivot test stand:
// derivative comes from the GYRO RATE (clean, no kick), so start with no integral
// (Ki 0) and tune Kp/Kd first, then add a little Ki only if there's steady-state lean.
constexpr float KP_PITCH = 0.4f, KI_PITCH = 0.0f, KD_PITCH = 0.15f;  // TUNE on stand
constexpr float KP_ROLL   = 0.4f, KI_ROLL   = 0.0f, KD_ROLL   = 0.15f;  // TUNE on stand
// Integrator anti-windup: clamp the integral term's contribution, in degrees. The
// attitude PID also freezes integration when its output saturates at TVC_CLAMP_DEG.
constexpr float PID_I_CLAMP_DEG = 6.0f;

// Altitude/throttle PID — output is a throttle FRACTION (0..1) added on top of
// the hover feed-forward (THROTTLE_HOVER), so the PID only trims the deviation
// and these stay small. Starting values; TUNE on the test stand (fan clamped).
constexpr float KP_ALT = 0.40f;
constexpr float KI_ALT = 0.08f;
constexpr float KD_ALT = 0.20f;
constexpr float ALT_I_CLAMP = 0.15f;   // integral term cap, throttle fraction

// ────────────────────────── Sensor fusion ───────────────────────────────────
constexpr float FUSION_ALPHA   = 0.98f; // gyro/accel complementary (attitude)
constexpr float ALT_BARO_ALPHA = 0.98f; // baro vs accel-integrated altitude
constexpr float ALT_TOF_BETA   = 0.70f; // ToF weight near ground
constexpr float TOF_MAX_RANGE_M    = 4.0f;  // VL53L1X usable range
constexpr float TOF_TILT_LIMIT_DEG = 20.0f; // ignore ToF beyond this tilt

// ────────────────────────────── Battery (6S) ────────────────────────────────
constexpr uint8_t BATT_CELLS = 6;
// Voltage divider 100k(R1)/10k(R2) from pack+ to GND, junction → PIN_BATT_ADC.
// ratio = (R1+R2)/R2 = 11.0; V_pack = V_pin · ratio. Read via analogReadMilliVolts
// (the ESP32-S3's eFuse-calibrated mV), so this ratio is the SINGLE calibration
// knob — trim it until the reported pack voltage matches a multimeter (verified
// ~22.8 V reads ~2.07 V at the junction). 25.2 V max → 2.29 V at the pin (≤3.3 V).
constexpr float BATT_DIVIDER_RATIO = 12.96f;  // 100k/10k nominal 11.0; trimmed to multimeter (12.44 read 24.0V @ true 25.0V → ×25.0/24.0)
constexpr uint8_t BATT_ADC_SAMPLES = 5;      // averaged per read to cut ADC noise
// Thrust scales with pack voltage (same µs = less thrust as it drains). Open-loop
// throttle (first hops, no altitude feedback) compensates for this explicitly — see
// THROTTLE_V_REF_V / THROTTLE_VCOMP_EXP above. The protection thresholds below are
// the hard safety net that always overrides compensation.
//
// 6S LiPo reference points (sourced — do not deviate): 25.2 V full (4.2 V/cell;
// NEVER charge above), 22.2 V nominal (3.7/cell), 18.0 V absolute damage line
// (3.0/cell; permanent damage + fire risk below). Protection thresholds are PACK
// voltages — the single divider on PIN_BATT_ADC reads PACK TOTAL only. They sit
// ABOVE a naive per-cell floor on purpose: cells drain NON-UNIFORMLY (the total can
// read fine while the weakest cell is already near 3.0 V), the discharge curve is
// flat then CLIFFS near empty, and the pack SAGS under load (~65 A) and rebounds at
// rest. So cut on the live under-load reading with margin, and LATCH the protection
// (lib/battery) — a rest rebound must never re-spin the fan.
constexpr float BATT_FLOOR_V = 20.5f;  // HARD CUT to ESC_MIN; never command thrust below this
constexpr float BATT_RAMP_V  = 21.0f;  // begin smooth powered ramp-down (above the floor)
constexpr float BATT_FULL_V  = 25.2f;  // 4.2 V/cell, full-charge ref for the % display
constexpr uint32_t BATT_RAMP_DOWN_MS = 1500; // smooth powered ramp-down duration

// ────────────────────────────── Landing ─────────────────────────────────────
// Landing is entirely PROPULSIVE — no parachute, no pyro. The throttle DESCEND
// phase ramps the setpoint down and cuts the motor at ALT_LANDED_M (see the
// flight-profile block above). There is nothing to deploy.

// ───────────────────────── Safety cutoffs (override) ────────────────────────
// Hard failsafe (lib/safety): in flight, ANY of these cuts the ESC to minimum.
constexpr float    SAFETY_MAX_TILT_DEG = 45.0f;  // tilt past this off vertical → cut
constexpr float    SAFETY_MAX_ALT_M    = 30.0f;  // runaway altitude cap (not yet wired)
constexpr uint32_t SAFETY_SENSOR_TIMEOUT_MS = 200; // IMU/ToF silent this long → cut
// Loop watchdog: a monitor task on the other core cuts the ESC if the control
// loop stops feeding it for this long (a stalled/hung loop).
constexpr uint32_t SAFETY_LOOP_WATCHDOG_MS  = 250; // ~12 missed 50 Hz ticks

// ───────────────────────────── Telemetry (WiFi) ─────────────────────────────
// SoftAP hosting the served control/telemetry page (lib/wifi_link), reachable at
// http://192.168.4.1 once joined.
constexpr char     WIFI_AP_SSID[] = "Zephyr-Telemetry";
constexpr char     WIFI_AP_PASS[] = "zephyr-flight";  // 8+ chars for WPA2

// ──────────────────────────────── OLED ──────────────────────────────────────
constexpr uint8_t OLED_WIDTH  = 128;
constexpr uint8_t OLED_HEIGHT = 64;

}  // namespace cfg
