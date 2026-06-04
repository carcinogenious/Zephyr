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
- **Thrust vector control** — a 2-axis gimbaled nozzle tilts ±12° on two MG90S
  servos to keep the rocket vertical, even at low speed where fins do nothing.
- **ESC throttle with arming** — thrust is commanded over an ESC PWM signal
  (1000–2000µs), with a boot-time arming sequence before the fan can spin.
- **Sensor fusion** — barometer, IMU, and a downward laser rangefinder fused for
  robust altitude and attitude estimation.
- **WiFi telemetry** — the board hosts a SoftAP and streams flight data over UDP
  to a ground laptop. One ESP32, no second radio, no second board.
- **Autonomous state machine** — arms, launches, flies vertical under TVC, then
  deploys a parachute at apogee for recovery.

## How it relates to SPARC

Zephyr deliberately shares SPARC's firmware conventions so modules port between
them. The difference is propulsion:

| | SPARC | Zephyr |
|---|---|---|
| Lift | 88g CO2 cartridge (~850 psi) | 70mm 6S EDF (~2240 g thrust) |
| Throttle | servo + ball valve | ESC PWM signal |
| Battery | 2S LiPo | 6S LiPo |
| Recovery | propulsive soft landing | parachute at apogee |
| Shared | Heltec WiFi LoRa 32 V3 · BMP388 + MPU6050 + VL53L1X · WiFi SoftAP telemetry · ±12° 2-axis TVC · OLED |

## Project structure

```
zephyr/
├── firmware/      ESP32 flight controller (PlatformIO)
│   ├── include/   config.h — all pins, gains, limits
│   ├── src/       main.cpp — entry point
│   └── lib/       modules: sensors, fusion, pid, tvc, throttle,
│                  wifi_link, display, state_machine, safety
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
| Telemetry | WiFi SoftAP + UDP to ground laptop |
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

To watch telemetry, join the board's `Zephyr-Telemetry` SoftAP from a laptop and
record the stream: `nc -ul 4210 > data/flights/$(date +%F)-hop.csv`.

## Flight profile

| Phase | Throttle | Control |
|-------|----------|---------|
| Arm | ESC armed (idle) | nozzle centered, OLED status on pad |
| Launch | ramp up | TVC active, hold vertical |
| Flight | launch profile | TVC active, telemetry downlink |
| Apogee | cut | deploy parachute (baro + IMU detect) |
| Descent / landed | off | recovery; log dump |

## Safety

- 6S at ~65A is serious power — bench-test the fan **only when clamped down**.
- Get the TVC deflection sign correct on a test stand **before any free flight**;
  a reversed sign steers the rocket into the ground.
- Arm the ESC only with the airframe restrained.
- LiPo: balance-charge only, on a non-flammable surface, never unattended.

Full procedures in [`docs/build-guide.md`](docs/build-guide.md).

## License

MIT — educational project.
