# Zephyr — EDF TVC Rocket

Thrust-vectored **electric ducted fan** rocket on a Heltec WiFi LoRa 32 V3
(ESP32-S3). A 70mm 6S EDF provides lift; a gimbaled exhaust nozzle on 2× MG90S
servos provides two-axis thrust vectoring (±20° mechanical, clamped to
`cfg::TVC_CLAMP_DEG` = ±10° during tuning); landing is entirely propulsive —
a controlled powered descent to a soft touchdown (no parachute).

This is the **parallel build to SPARC** (the CO2 rocket). The flight computer,
sensor suite, WiFi telemetry, TVC logic, and `firmware/lib/` module layout are
intentionally shared with SPARC so code ports between the two. **What differs is
propulsion** — see below. When in doubt, match SPARC's conventions.

> Keep this file short. It points to the docs; it does not duplicate them.
> The full spec is in [`docs/build-guide.md`](docs/build-guide.md).

## How Zephyr differs from SPARC
| | SPARC | Zephyr |
|---|---|---|
| Propulsion | 88g CO2 cartridge, ~850 psi | 70mm 6S EDF, ~2240 g thrust |
| Throttle | servo-actuated ball valve | **ESC PWM** (needs an arming sequence) |
| Throttle servo | MG996R (3rd servo) | none — throttle is the ESC signal |
| Battery / bus | 2S 7.4V | **6S 22.2V** (fat path → ESC; 5V BEC → logic) |
| All-up mass | ~670–700 g | ~970–1000 g |
| Hazards | high-pressure gas, cryogenic CO2 | **high current (65A), high-RPM fan** |
| Same | Heltec V3, BMP388/MPU6050/VL53L1X, WiFi SoftAP telemetry, 2-axis TVC gimbal, OLED, **propulsive soft landing (no parachute)** |

## Build & flash
```bash
pio run                       # build
pio run -t upload             # flash the Heltec over USB
pio device monitor -b 115200  # serial monitor
```
PlatformIO project is under `firmware/`; `platformio.ini` at the repo root remaps
`src_dir`/`lib_dir`/`include_dir` into it. The `[env]` + `lib_deps` block is filled
in during bring-up (same library set as SPARC — see build-guide Part B Step 1).

## Current firmware (Phase 1)
`src/main.cpp` runs a fixed **100 Hz** (`cfg::CONTROL_LOOP_HZ`) loop with two
independent controllers on independent actuators:
- **Attitude PID → 2 TVC servos** (hold vertical). Per-axis, setpoint 0°, derivative
  from the **gyro rate** (not differenced angle), conditional anti-windup, output
  clamped to `cfg::TVC_CLAMP_DEG`. Live from boot; verify the per-axis SIGN
  (`TVC_*_DIR`) on the stand. Roll is uncontrolled (2-axis nozzle).
- **Throttle → ESC**, web **manual slider** (open-loop): slider 0..1 → voltage-
  compensate → battery protection → ESC. Gated behind arm+launch; `MANUAL` phase.

Phased plan: **P1 (now)** attitude PID + manual/open-loop throttle. **P2 (later,
deferred)** altitude fusion → closed-loop height; `throttle::update()`/`beginProfile()`
already exist for it. Baro reads at ~20 Hz (display only, NOT used for altitude); ToF
is the relative-height source (≈0 at the pad). WiFi runs on core 0, the loop on core
1, so the slider can never make the attitude loop laggy.

## Code conventions
- **All hardware constants live in `firmware/include/zephyr_config.h`** — GPIO pins,
  I2C addresses, PID gains, the deflection clamp (`TVC_CLAMP_DEG`), throttle min/max µs, ESC
  arming duration, battery divider + thresholds. One place to tune; nothing magic
  buried in logic.
- **One subsystem = one library** under `firmware/lib/`. `src/main.cpp` only wires
  modules together in `setup()`/`loop()`.
  Modules: `sensors` (bmp388/mpu6050/tof), `fusion`, `pid`, `tvc`, `throttle`,
  `battery`, `wifi_link`, `display`, `state_machine`, `safety`.
- **ESC + both servos run on native LEDC** via `ledcAttachChannel(pin, 50, 14, ch)`,
  sharing one 50 Hz timer on **explicit channels 0 (ESC) / 1 (pitch) / 2 (roll)**.
  Write with `ledcWrite(pin, usToDuty(us))`, `usToDuty = (us << 14) / 20000`. **Attach
  each channel ONCE in setup** (its module's `init()`) — re-attaching returns `false`.
  This is the **only proven 3-PWM arrangement** on this S3 + Arduino core 3.x.
  ⚠️ **Do NOT regress to** (all tried and FAILED here): ESP32Servo for all three — its
  shared MCPWM timers collide on the S3 (only 2 of 3 outputs work; the ESC gets
  corrupted into an uncommanded spin, and `allocateTimer(0..3)` does not fix it); the
  Dlloydev ESP32-AnalogWrite lib — uses the old `ledcSetup`/`ledcAttachPin` API and
  won't compile on core 3.x; raw `ledcAttach` (auto-channel) — returned fail, use
  explicit channels; calling `ledcAttachChannel` in `loop()` — returns `false`.
- **ESC throttle-range calibration** (root cause of "won't spin below ~1610 µs"): if
  the ESC only responds in a compressed top range, re-run its calibration — hold MAX
  (2000 µs) 6 s while connecting the battery at power-up, then hold MIN (1000 µs) 10 s.
  After this it spins across the full 1000–2000 µs range. No power cycle needed.
- **TVC servo scaling** (measured, mounted): centers PITCH 1600 µs / ROLL 1700 µs
  (different is normal — per-axis constants absorb it, do not force 1500). Both axes
  are linear and **symmetric at 30 µs/deg**: `us = center + deg·30`. Servos read the
  3.3 V signal directly — **no level shifter on the servos**, only on the ESC.
- **Smooth servo motion (required for clean thrust vectoring):** command the target
  pulse directly and let the servo slew to it **natively** — the analog servo's own
  motion is the smooth glide. Do **NOT** software-interpolate between positions (it's
  slow and visibly steppy), and do **NOT** re-issue the same pulse every loop (it
  buzzes). `lib/tvc` writes on-change only. The PID feeds it a smoothly-varying target
  each tick; let the servo track it.
- **Conventional Commits**: `type(scope): summary`, where `scope` is a module name
  above (or `main`). Branch off `main`; confirm `pio run` builds before committing.
  Commit/push only when asked.

## Pin map (defined in `firmware/include/zephyr_config.h`)
Working defaults, reusing SPARC's verified safe pins. The ESC signal goes out on
GPIO 6 through a **74AHCT125N** buffer that lifts the 3.3V PWM to the 5V the ESC
needs to arm (transparent to firmware — see [`docs/wiring-diagram.md`](docs/wiring-diagram.md)).

| Pin | Function |
|-----|----------|
| GPIO 41 | I2C SDA (BMP388 + MPU6050 + VL53L1X) |
| GPIO 42 | I2C SCL |
| GPIO 6  | ESC throttle signal (1000–2000µs) **via 74AHCT125N 3.3→5V buffer**; 10k pulldown |
| GPIO 48 | TVC pitch servo (MG90S) |
| GPIO 5  | TVC roll servo (MG90S) |
| GPIO 1  | Battery ADC — **6S divider** 100k/10k, ratio 11 (25.2V max → 2.29V); not SPARC's 2S divider |
| GPIO 26 | Free spare (was recovery — none now; propulsive landing) |
| GPIO 2 / 19 / 20 | Free spares (no arm button / buzzer / LED — web UI handles all of it) |
| — | OLED SSD1306: on-board, BSP macros (SDA_OLED/SCL_OLED/RST_OLED), 0x3C |

**Heltec V3 (ESP32-S3FN8) pin cautions:** GPIO 8–14 are the on-board SX1262 LoRa
SPI (radio left disabled). GPIO 26–32 are the module SPI-flash interface (26 usable
as a spare only because there's no PSRAM). GPIO 0/3/45/46 are strapping/boot pins
(GPIO 3 is JTAG-select on the S3 — not suitable for a servo/sensor; roll is on
GPIO 5). GPIO 47/48 sit next to the USB-UART lines and are noisy for continuous
PWM — the ESC was moved off 47 to GPIO 6 (GPIO 6/7 are ordinary free GPIOs on the
S3, *not* SPI-flash pins as on the classic ESP32). Avoid GPIO 4 (wiring fault). Do
not reassign the OLED BSP pins.

## Safety-critical invariants (do not violate)
- **Nozzle deflection clamp** (`cfg::TVC_CLAMP_DEG`, currently ±10° for tuning;
  mechanical max ±20°) always enforced in `lib/tvc`, as the PRIMARY limit, with a
  per-axis hard µs backstop behind it.
- **ESC↔servo PWM coexistence is SOLVED** — all three run on native LEDC via
  `ledcAttachChannel`, one shared 50 Hz timer, explicit channels 0/1/2, attached once
  in setup (see conventions). Do NOT regress to ESP32Servo for all three (the S3 MCPWM
  collision that caused the uncommanded spin). The ESC and servos still keep SEPARATE
  write paths (`throttle::*` vs `tvc::*`) — no shared "write all PWM" loop.
- **Throttle stays software arming-gated** (forced to ESC min unless armed) — kept as
  defense-in-depth even though the LEDC channels now isolate the outputs in hardware.
- **6S pack floor is 20.5 V** (`cfg::BATT_FLOOR_V`), NOT a naive 3.0 V/cell (18 V):
  one divider reads pack total only, cells drain unevenly, and the curve cliffs near
  empty — 20.5 V keeps the weakest cell safe. Hard-cut at the floor, ramp-down above
  it, on the LIVE under-load reading; protection LATCHES (no re-spin on rest rebound)
  and **always overrides** the open-loop voltage compensation.
- **Common ground** across ESC, ESP32, servos, and sensors — a floating ground is
  the #1 first-build failure.
- **Fan CLAMPED** for every ESC/throttle bench test. 6S at ~65A spins the fan to
  tens of thousands of RPM.
- **Verify the TVC deflection sign on the test stand before any free flight** — a
  reversed sign steers the rocket into the ground.
- Arm the ESC only with the airframe restrained.
- 6S LiPo: balance-charge only, on a non-flammable surface, never unattended.

## Docs
- [`docs/build-guide.md`](docs/build-guide.md) — full hardware + firmware spec
- [`docs/wiring-diagram.md`](docs/wiring-diagram.md) — power rail, grounds, pin map
- [`docs/mass-budget.md`](docs/mass-budget.md) — weights, T/W
- [`docs/bom.md`](docs/bom.md) — bill of materials
- [`docs/flight_logs/`](docs/flight_logs/) — one file per test/flight
