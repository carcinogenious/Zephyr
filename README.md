# Zephyr

**Thrust-vectored electric ducted fan rocket**

A 70mm EDF rocket that actively stabilizes its own flight with a gimbaled exhaust
nozzle, flown by an ESP32 flight computer with live WiFi telemetry. Zephyr is the
**electric counterpart to [SPARC](../SPARC)** — same flight computer, sensors, TVC
logic, and telemetry, but lifted by a rechargeable ducted fan instead of a CO2
cartridge.

## What makes this different

Most model rockets are ballistic — light the motor, hope for the best. Zephyr
controls its flight in real time, and does it on electric power:

- **Electric ducted fan** — a 70mm 6S EDF (~2240 g thrust) means no consumables,
  no pressurized gas, and a throttle you can hold steady. Recharge and re-fly.
- **Thrust vector control** — a 2-axis gimbaled nozzle (±20° mechanical, clamped to
  ±10° during tuning) on two MG90S
  servos to keep the rocket vertical, even at low speed where fins do nothing.
- **ESC throttle with arming** — thrust is commanded over an ESC PWM signal
  (1000–2000µs), with a boot-time arming sequence before the fan can spin.
- **Sensor fusion** — barometer, IMU, and a downward laser rangefinder fused for
  robust altitude and attitude estimation.
- **WiFi telemetry + control page** — the board hosts its own access point and
  serves a phone/laptop page with live telemetry and arm / launch / abort
  buttons. The radio runs in a separate task and can never gate the flight loop.
  One ESP32, no second radio, no second board.
- **Autonomous state machine** — arms, launches, flies vertical under TVC, hovers,
  then lands itself propulsively under power (no parachute).

## How it relates to SPARC

Zephyr deliberately shares SPARC's firmware conventions so modules port between
them. The difference is propulsion:

| | SPARC | Zephyr |
|---|---|---|
| Lift | 88g CO2 cartridge (~850 psi) | 70mm 6S EDF (~2240 g thrust) |
| Throttle | servo + ball valve | ESC PWM signal |
| Battery | 2S LiPo | 6S LiPo |
| Shared | Heltec WiFi LoRa 32 V3 · BMP388 + MPU6050 + VL53L1X · WiFi SoftAP telemetry · 2-axis TVC · OLED · propulsive soft landing |

## Project structure

```
zephyr/
├── firmware/      ESP32 flight controller (PlatformIO)
│   ├── include/   zephyr_config.h — all pins, gains, limits
│   ├── src/       main.cpp — entry point
│   └── lib/       modules: sensors, fusion, pid, tvc, throttle,
│                  battery, wifi_link, display, state_machine, safety
├── web/           served control/telemetry page (index.html)
├── cad/           airframe + mounts, STLs in cad/stl/
├── docs/          build guide, wiring, mass budget, BOM, flight logs
├── data/flights/  recorded telemetry (.csv)
└── platformio.ini
```

See [`docs/build-guide.md`](docs/build-guide.md) for the complete hardware +
firmware spec, and [`CLAUDE.md`](CLAUDE.md) for build commands and conventions.

## Hardware

| Subsystem | Key component |
|-----------|---------------|
| Propulsion | Powerfun 70mm 6S 2300kv EDF, ~2240 g thrust |
| ESC | 80A, 2–6S, 5V/5A BEC |
| Battery | OVONIC 6S 1300mAh 120C LiPo |
| TVC | 3D-printed gimbaled nozzle + 2× MG90S servos + 1.2mm steel pushrods |
| Sensors | BMP388 (baro) + MPU6050 (IMU) + VL53L1X (ToF) |
| Computer | Heltec WiFi LoRa 32 V3 (ESP32-S3) |
| Telemetry | WiFi SoftAP + served control/telemetry web page |
| Power | one 6S battery; ESC BEC feeds a shared 5V rail (common ground) |

All-up weight ~970–1000 g · bench T/W ~2.2 (~1.9 with intake loss) · hover ~45%.

## Development

### Prerequisites
- [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)
- A 1S–6S LiPo balance charger
- A 3D printer for the nozzle + servo bracket (`cad/stl/`)

### Quick start
```bash
git clone <repo-url> zephyr
cd zephyr
# PlatformIO auto-installs dependencies on first build
pio run                       # build
pio run -t upload             # flash the Heltec over USB
pio device monitor -b 115200  # serial monitor
```

### Telemetry & control page

The board hosts a WiFi access point and serves a control/telemetry page:

1. Join the AP **`Zephyr-Telemetry`** (password `zephyr-flight`).
2. Open **http://192.168.4.1** in a browser.

The page shows live telemetry (vertical height, attitude, throttle, phase,
sensor status) and has **ARM / LAUNCH / ABORT** buttons. After landing it shows a
flight summary to paste into `docs/flight_logs/`. The page source is
[`web/index.html`](web/index.html) (the firmware serves an embedded copy) — you
can also open that file directly while joined to the AP.

## Flight profile

| Phase | Throttle | Control |
|-------|----------|---------|
| Arm | ESC armed (idle) | nozzle centered, OLED status on pad |
| Launch | ramp up | TVC active, hold vertical |
| Ascend / hover | launch profile | TVC active, telemetry downlink |
| Descent | powered down-ramp | TVC active, controlled descent on ToF |
| Landed | cut | motor off; soft touchdown, log dump |

## Safety

- 6S at ~65A is serious power — bench-test the fan **only when clamped down**.
- Get the TVC deflection sign correct on a test stand **before any free flight**;
  a reversed sign steers the rocket into the ground.
- Arm the ESC only with the airframe restrained.
- LiPo: balance-charge only, on a non-flammable surface, never unattended.

Full procedures in [`docs/build-guide.md`](docs/build-guide.md).

## License

MIT — educational project.
