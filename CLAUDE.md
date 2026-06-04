# Zephyr — EDF TVC Rocket

Thrust-vectored **electric ducted fan** rocket on a Heltec WiFi LoRa 32 V3
(ESP32-S3). A 70mm 6S EDF provides lift; a gimbaled exhaust nozzle on 2× MG90S
servos provides ±12° two-axis thrust vectoring; recovery is a parachute deployed
at apogee.

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
| Recovery | propulsive hover + soft landing | **parachute at apogee** |
| All-up mass | ~670–700 g | ~970–1000 g |
| Hazards | high-pressure gas, cryogenic CO2 | **high current (65A), high-RPM fan** |
| Same | Heltec V3, BMP388/MPU6050/VL53L1X, WiFi SoftAP telemetry, ±12° gimbal, OLED |

## Build & flash
```bash
pio run                       # build
pio run -t upload             # flash the Heltec over USB
pio device monitor -b 115200  # serial monitor
```
PlatformIO project is under `firmware/`; `platformio.ini` at the repo root remaps
`src_dir`/`lib_dir`/`include_dir` into it. The `[env]` + `lib_deps` block is filled
in during bring-up (same library set as SPARC — see build-guide Part B Step 1).

## Code conventions
- **All hardware constants live in `firmware/include/config.h`** — GPIO pins, I2C
  addresses, PID gains, the ±12° deflection limit, throttle min/max µs, ESC arming
  duration. One place to tune; nothing magic buried in logic.
- **One subsystem = one library** under `firmware/lib/`. `src/main.cpp` only wires
  modules together in `setup()`/`loop()`.
  Modules: `sensors` (bmp388/mpu6050/tof), `fusion`, `pid`, `tvc`, `throttle`,
  `wifi_link`, `display`, `state_machine`, `safety`.
- Use **ESP32Servo** (not Arduino Servo) for both servos AND the ESC.
- **Conventional Commits**: `type(scope): summary`, where `scope` is a module name
  above (or `main`). Branch off `main`; confirm `pio run` builds before committing.
  Commit/push only when asked.

## Pin map (defined in `firmware/include/config.h`)
Working defaults, reusing SPARC's verified safe pins. The third servo pin (SPARC's
throttle valve) becomes Zephyr's ESC signal.

| Pin | Function |
|-----|----------|
| GPIO 41 | I2C SDA (BMP388 + MPU6050 + VL53L1X) |
| GPIO 42 | I2C SCL |
| GPIO 47 | ESC throttle signal (1000–2000µs) |
| GPIO 48 | TVC pitch servo (MG90S) |
| GPIO 3  | TVC yaw servo (MG90S) |
| GPIO 2  | Arm/launch button (INPUT_PULLUP) |
| GPIO 1  | Battery ADC — **6S divider** (25.2V max → 3.3V; not SPARC's 2S divider) |
| GPIO 26 | Recovery deploy trigger (servo / MOSFET) |
| GPIO 19 / 20 | Buzzer / status LED |
| — | OLED SSD1306: on-board, BSP macros (SDA_OLED/SCL_OLED/RST_OLED), 0x3C |

**Heltec V3 (ESP32-S3FN8) pin cautions:** GPIO 6/7 are the module SPI flash — never
use. GPIO 8–14 are the on-board SX1262 LoRa SPI (radio left disabled). GPIO 26–32
are the SPI-flash interface (26 usable as a spare only because there's no PSRAM).
GPIO 0/45/46 are strapping/boot pins. Do not reassign the OLED BSP pins.

## Safety-critical invariants (do not violate)
- **±12° deflection clamp** on the nozzle, always, in `lib/tvc`.
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
