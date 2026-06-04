# Zephyr — EDF TVC Rocket — Build Guide

A thrust-vectored electric-ducted-fan rocket using a 70mm EDF, gimbaled exhaust
nozzle, and an ESP32 (Heltec WiFi LoRa 32 V3) flight computer. This is the
backup/parallel build to the CO2 rocket (SPARC) — same flight computer, same TVC
logic, same telemetry approach, different propulsion.

Zephyr deliberately mirrors SPARC's firmware conventions so the two codebases stay
easy to cross-reference and share modules. Where the two differ, it is because the
propulsion differs (EDF + ESC instead of a CO2 ball valve), never for style.

---

## 0. Design Summary (locked decisions)

| Item | Choice |
|---|---|
| Propulsion | Powerfun 70mm 6S 2300kv EDF, ~2240g max thrust, ~178g |
| ESC | 80A, 2–6S, with 5V/5A BEC, XT60 |
| Battery | OVONIC 6S 1300mAh 120C, XT60, ~232g |
| Charger | 1S–6S balance charger |
| Flight computer | Heltec WiFi LoRa 32 V3 (ESP32-S3) — ONE board, no second board |
| Sensors | BMP388 (baro), MPU6050 (IMU), VL53L1X (ToF) — all I2C |
| TVC | Gimbaled exhaust nozzle, 2× MG90S servos |
| Telemetry | WiFi SoftAP + UDP to a ground laptop (LoRa radio present but unused) |
| All-up weight | ~970–1000 g |
| T/W (bench) | ~2.2 |
| T/W (real, 15% intake loss) | ~1.9 |
| Hover throttle | ~45% |

**Power architecture:** one 6S battery. Fat path (22.2V) → ESC → fan. BEC path
(ESC steps 22.2V → 5V) → ESP32 + servos. Sensors run on ESP32's own 3.3V.
Everything shares a common ground.

**Telemetry architecture:** one ESP32. The board hosts a WiFi SoftAP
("Zephyr-Telemetry") and streams flight data over UDP. A ground-station **laptop**
joins the AP and records the stream (`nc -ul 4210 > flight.csv`) — there is no
second flight computer and no LoRa link. The on-board SX1262 LoRa radio is left
disabled (enabling it locks its internal SPI GPIOs that we want free for spares).

---

## PART A — HARDWARE BUILD

Order matters: build and verify the electronics on the bench BEFORE anything goes
in the airframe. Never fly anything you haven't bench-tested.

### Step 1 — Bench-test the bare electronics (no power electronics yet)
1. Flash the Heltec with a "hello world" that drives the OLED and scans I2C.
2. Wire the three sensors to the I2C bus (SDA/SCL + 3.3V + GND from the ESP32).
3. Confirm all three addresses appear on an I2C scan: 0x76/0x77 (BMP388),
   0x68 (MPU6050), 0x29 (VL53L1X).
4. Confirm each sensor returns sane data (altitude, accel/gyro, range).
   *Do not proceed until all three read correctly.*

### Step 2 — Build the 5V power rail
You have ONE 5V source (the ESC's BEC) feeding THREE loads (ESP32 + 2 servos).
Build a shared rail — do not daisy-chain.
1. Make a small "+5V" node and a "GND" node (perfboard strip, or a soldered star,
   or a servo Y-harness).
2. BEC red (5V) → +5V node. BEC black (GND) → GND node.
3. From +5V node: branch to ESP32 5V pin, servo1 +, servo2 +.
4. From GND node: branch to ESP32 GND, servo1 −, servo2 −, AND sensor GND.
5. BEC current budget: 5V/5A. ESP32 ~0.3A + 2× MG90S ~1.4A peak = ~1.7A. Big margin.

**Critical:** every ground in the system ties to the GND node — ESC, ESP32,
servos, sensors. A floating ground = jittery/dead PWM. This is the #1 first-build
failure.

### Step 3 — ESC + motor bench test (FAN RESTRAINED)
1. Connect ESC three phase wires to the EDF motor (bullet plugs).
2. ESC signal wire → designated ESP32 GPIO (throttle).
3. ESC power → 6S battery via XT60. **Do not plug battery in yet.**
4. **Clamp the EDF unit to the bench. Clear all loose wires, fingers, hair.**
   At 65A this fan spins to tens of thousands of RPM.
5. Upload firmware with the arming sequence (see Part B, Step 3).
6. Plug in battery. Confirm ESC arming beeps, then test low-throttle spin-up.
7. Verify throttle response across the range. Listen for imbalance/rubbing.
   *Keep the unit clamped the entire time.*

### Step 4 — Print the TVC parts
1. Print the gimbaled nozzle (HoloModels 70mm TV nozzle STL, or Matzetas 360°
   unit scaled to 70mm). Printable parts live in `cad/stl/`.
2. Print settings: 2 wall lines (0.8mm) minimum for rigidity, parts upright,
   no support where possible.
3. **Sand the ball-joint contact surfaces** and apply PTFE dry lube — a sticky
   joint is the most common TVC failure.
4. Print the servo bracket / gimbal mount that holds the 2 MG90S servos relative
   to the nozzle.

### Step 5 — Assemble the TVC stage
1. Mount the EDF unit fixed in the lower airframe.
2. Mount the gimbaled nozzle in the exhaust, just aft of the fan.
3. Mount the 2 servos in the bracket: servo1 = pitch, servo2 = yaw.
4. Connect servo horns to nozzle arms with 1.2mm steel pushrods.
5. With servos powered and centered (1500µs), adjust pushrod length so the
   nozzle sits exactly centered/straight. This is your mechanical zero.
6. Confirm ±12° travel in pitch and yaw with no binding through full range.

### Step 6 — Airframe integration (layout top→bottom)
```
[ Nose cone + parachute + recovery ]
[ Payload bay: Heltec + 3 sensors   ]   ← electronics
[ Air intake holes (ring of holes)  ]   ← total inlet area >= fan area (~38 cm2 for 70mm)
[ 6S battery                        ]   ← heavy item, mid-body
[ Open duct down to fan             ]
[ EDF unit (fixed)                  ]   ← heavy item, low
[ Gimbal + TVC nozzle               ]   ← pivot, very bottom
```
1. Mount battery mid-body (sets CoM near center → long lever arm to nozzle).
2. Route ALL wiring on the OUTSIDE of the airframe in a channel — keep the
   internal duct clean so airflow to the fan is uninterrupted.
3. Mount VL53L1X facing straight down (altitude-above-ground / landing).
4. Verify CoM is above the nozzle pivot by a healthy margin (longer = more
   control authority).

### Step 7 — Recovery
1. Parachute sized for ~1 kg at a safe descent rate.
2. Deployment triggered by the flight computer (apogee detect via baro + IMU).
3. Bench-test the deployment mechanism independently before flight.

---

## PART B — SOFTWARE / FIRMWARE

Reuse as much of the SPARC firmware as possible — the module layout under
`firmware/lib/` is intentionally the same. The TVC control loop is identical; only
the throttle output changes (servo valve → ESC) and you add an ESC arming
sequence. Telemetry is WiFi SoftAP + UDP, exactly as on SPARC.

### Step 1 — Toolchain
1. Arduino IDE or PlatformIO with ESP32-S3 board support (Heltec WiFi LoRa 32 V3).
2. Libraries (same set as SPARC):
   - `adafruit/Adafruit BMP3XX Library`
   - `adafruit/Adafruit MPU6050`
   - `adafruit/Adafruit Unified Sensor`
   - `pololu/VL53L1X`
   - `madhephaestus/ESP32Servo` (NOT the Arduino Servo library)
   - `thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays`
   WiFi is built into the ESP32 Arduino framework — no library needed. No LoRa or
   SD libraries (LoRa radio left disabled; no SD card).
3. Use ESP32 hardware PWM via ESP32Servo for BOTH the servos AND the ESC —
   the ESC reads the same 1000–2000µs signal as a servo.

### Step 2 — Bring-up order (mirror the hardware steps)
There is no separate `test/` directory — bring-up is done by flashing the main
firmware incrementally, the same way SPARC does it. Bring each subsystem up in
this order and confirm it before moving on:
1. OLED + I2C scan → confirm board + bus.
2. Each sensor individually → confirm data.
3. Sensor fusion (`lib/fusion`): feed MPU6050 (primary attitude) into your filter;
   baro for altitude; ToF for ground proximity.
4. Servo sweep → confirm pitch/yaw map to nozzle correctly, set center.
5. ESC throttle test (FAN CLAMPED) → confirm arming + throttle range.
6. WiFi telemetry (`lib/wifi_link`) → confirm the laptop joins the SoftAP and
   receives the UDP stream.

### Step 3 — ESC arming sequence (the new piece vs the CO2 build)
An ESC will NOT spin until it sees a minimum-throttle signal at startup.
Fold this into your existing pre-flight/arm state:
1. On boot, hold ESC signal at minimum (1000µs) for ~2 seconds.
2. Wait for ESC arming confirmation (beeps).
3. Only then enter the ARMED state; show it on the OLED.
4. Throttle stays at minimum until launch command.

### Step 4 — Control loop (reuse from SPARC)
1. Read IMU → estimate attitude (pitch/yaw error from vertical).
2. PID per axis (`lib/pid`) → desired nozzle deflection.
3. Clamp deflection to ±12°, map to servo µs (`lib/tvc`), write to both servos.
4. Throttle (`lib/throttle`): open-loop profile (ramp to launch throttle) OR
   closed-loop on baro-derived velocity, your choice. Start open-loop.
5. Telemetry (`lib/wifi_link`): stream altitude, attitude, state, servo positions,
   throttle, battery voltage over WiFi/UDP. OLED blank during flight to save power.

### Step 5 — State machine (`lib/state_machine`)
```
BOOT → SENSOR_CHECK → ARM (ESC arming) → ARMED (on pad, OLED status)
     → LAUNCH (throttle up, TVC active)
     → FLIGHT (TVC active, telemetry)
     → APOGEE (deploy recovery)
     → DESCENT → LANDED (motor cut, log dump)
```

### Step 6 — Tuning (do NOT skip)
1. **Test stand:** pin the rocket at its CoM, free to rotate.
2. Tune PID gains until it holds vertical and rejects nudges without oscillating.
3. Verify nozzle direction is correct (positive error → correcting deflection,
   not runaway). Get the sign right BEFORE any free flight.
4. Only after stable test-stand behavior → first free hop, low altitude, open area.

---

## PART C — REPOSITORY STRUCTURE

Zephyr uses the same layout as SPARC: the PlatformIO project lives under
`firmware/`, with `platformio.ini` at the repo root remapping the source dirs into
it. This keeps the flight firmware cleanly separated from CAD, docs, and flight
data at the top level.

```
zephyr/
├── platformio.ini             # ROOT — project anchor; remaps src/lib/include into firmware/
├── CLAUDE.md                  # ROOT — agent instructions: build cmds, conventions, safety invariants
├── README.md                  # overview, status, build log links
├── LICENSE
├── .gitignore                 # ignores .pio/, build artifacts, .DS_Store, secrets
├── firmware/                  # the PlatformIO project (mirrors SPARC)
│   ├── include/
│   │   └── config.h           # PIN MAP, PID gains, limits, I2C addresses (shared header)
│   ├── src/
│   │   └── main.cpp           # entry point ONLY: setup/loop, wires modules together
│   └── lib/                   # all modules as project-private PlatformIO libraries
│       ├── sensors/           # bmp388 + mpu6050 + tof drivers (grouped, part-named)
│       │   ├── bmp388.cpp/.h
│       │   ├── mpu6050.cpp/.h
│       │   └── tof.cpp/.h
│       ├── fusion/            # complementary filter: attitude + altitude
│       │   └── attitude.cpp/.h
│       ├── pid/               # reusable PID controller (per-axis)
│       │   └── pid.cpp/.h
│       ├── tvc/               # nozzle mapping, ±12° deflection clamp, servo output
│       │   └── tvc.cpp/.h
│       ├── throttle/          # ESC arming + throttle profile (EDF-specific)
│       │   └── throttle.cpp/.h
│       ├── wifi_link/         # WiFi SoftAP + UDP telemetry downlink
│       │   └── wifi_link.cpp/.h
│       ├── display/           # on-board OLED pad-status screens
│       │   └── display.cpp/.h
│       ├── state_machine/     # flight state machine + transitions
│       │   └── state_machine.cpp/.h
│       └── safety/            # cutoff checks (tilt/alt/battery/sensor-timeout)
│           └── safety.cpp/.h
├── cad/                       # source CAD (airframe, mounts) + printables
│   └── stl/
│       ├── tvc-nozzle-70mm.stl
│       ├── servo-bracket.stl
│       └── ...
├── docs/
│   ├── build-guide.md         # this document
│   ├── wiring-diagram.md      # power rail, pin map, grounds
│   ├── mass-budget.md         # component weights, T/W calcs
│   ├── bom.md                 # bill of materials + purchase links
│   ├── reference/             # datasheets, EDF/ESC specs
│   └── flight_logs/           # one markdown file per test/flight
│       └── _TEMPLATE.md       # copy to YYYY-MM-DD-<name>.md per event
└── data/
    └── flights/               # raw telemetry logs (.csv) recorded by the ground laptop
```

**Why the `firmware/` wrapper (and `platformio.ini` at the root):** SPARC uses this
layout, and PlatformIO supports it via `src_dir`/`lib_dir`/`include_dir` keys in the
`[platformio]` section — the toolchain does NOT require `src/` to be a sibling of
`platformio.ini`. The wrapper keeps the firmware in one place, separate from `cad/`,
`docs/`, and `data/`.

**Why `src/` holds only `main.cpp`:** every subsystem is a self-contained library
under `firmware/lib/`, exactly as in SPARC. `main.cpp` just wires them together in
`setup()`/`loop()`. This enforces clean module boundaries and lets modules be
reused between the two rockets.

**Why `CLAUDE.md` is at the repo root:** agent tooling (Claude Code) auto-loads
`CLAUDE.md` from the repository root. Keep it short — it should POINT to the docs,
not duplicate them: toolchain/build commands, code conventions (all hardware
constants live in `firmware/include/config.h`), and the safety-critical invariants
(±12° deflection clamp, common ground, fan clamped during ESC tests, verify TVC
sign before free flight). It references `docs/build-guide.md` rather than containing
it.

**Key file: `firmware/include/config.h`** — keep ALL hardware-specific constants
here (GPIO pin numbers, I2C addresses, PID gains, ±12° deflection limit, throttle
min/max µs, arming duration). It lives in `include/` so every library and `main.cpp`
can include it. One place to tune, nothing magic buried in logic.

**No `test/` directory:** like SPARC, Zephyr does bench bring-up by flashing the
main firmware incrementally (see Part B, Step 2), not via the PlatformIO test
runner. Physical verification — I2C scan, servo sweep, FAN-CLAMPED ESC test — is
the real "test suite" and is captured in this guide and in `docs/flight_logs/`.

**No ground-station firmware:** there is only ONE ESP32. Telemetry goes out over a
WiFi SoftAP; a laptop joins it and records the UDP stream into `data/flights/`. No
second board, no receiver firmware to maintain.

**Commit conventions:** follow Conventional Commits (`type(scope): summary`) with
`scope` matching the `firmware/lib/` module names (`sensors`, `fusion`, `pid`,
`tvc`, `throttle`, `wifi_link`, `display`, `state_machine`, `safety`, or `main`),
consistent with SPARC.

---

## PART D — BUILD ORDER CHECKLIST (the "what goes when")

```
[ ] 1. Order parts (EDF, ESC, battery, charger, servos if needed)
[ ] 2. Bench: flash Heltec, OLED + I2C scan
[ ] 3. Bench: all 3 sensors reading correctly
[ ] 4. Build 5V power rail (common ground!)
[ ] 5. Bench: ESC + motor, FAN CLAMPED, arming + throttle
[ ] 6. Bench: WiFi SoftAP up, laptop receives UDP telemetry
[ ] 7. Print TVC nozzle + bracket, sand + lube ball joint
[ ] 8. Assemble TVC stage, set mechanical zero, verify ±12°
[ ] 9. Firmware: ESC arming sequence into pre-flight state
[ ] 10. Firmware: port TVC control loop, verify deflection sign
[ ] 11. Airframe integration (battery mid-body, wiring external, clean duct)
[ ] 12. Recovery system bench-tested independently
[ ] 13. TEST STAND: pinned at CoM, PID tuning until stable
[ ] 14. First free hop — low, open area, restrained until arm
```

Active build time: ~1 week of evenings/weekend once parts arrive; calendar time
gated by shipping (1–2 weeks). PID tuning on the stand eats the most time — budget
for it, never skip it.

---

## SAFETY NOTES
- 6S at 65A is serious power. Bench-test the fan ONLY when clamped down.
- LiPo: charge on a non-flammable surface, never unattended, balance-charge only.
- Get the TVC deflection sign correct on the test stand before any free flight —
  a reversed sign means the rocket actively steers itself into the ground.
- Arm the ESC only with the airframe restrained.
