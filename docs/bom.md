# Zephyr — Bill of Materials

Parts list for the EDF TVC rocket. Specs and quantities are locked from the design
summary; **purchase links and prices are TBD** — fill them in as you source parts.

> Some items may already be on hand (shared with the SPARC build: Heltec board,
> the three sensors, MG90S servos). Mark those rather than re-buying.

## Propulsion & power

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 1 | EDF unit | Powerfun 70mm, 6S, 2300kv, ~2240 g thrust, ~178 g | 1 | TBD | TBD | the motor + fan |
| 2 | ESC | 80A, 2–6S, built-in 5V/5A BEC, XT60 | 1 | TBD | TBD | signal = servo-style PWM |
| 3 | Battery | OVONIC 6S 1300mAh 120C LiPo, XT60, ~232 g | 1 | TBD | TBD | |
| 4 | Balance charger | 1S–6S | 1 | TBD | TBD | balance-charge only |
| 5 | XT60 connectors | male/female pairs | 2–3 | TBD | TBD | battery + ESC + harness |

## Flight computer & sensors

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 6 | Flight computer | Heltec WiFi LoRa 32 V3 (ESP32-S3) | 1 | TBD | TBD | OLED on-board; shared w/ SPARC |
| 7 | Barometer | BMP388, I2C 0x76/0x77 | 1 | TBD | TBD | |
| 8 | IMU | MPU6050 (GY-521), I2C 0x68 | 1 | TBD | TBD | primary attitude |
| 9 | Rangefinder | VL53L1X (TOF400C), I2C 0x29 | 1 | TBD | TBD | mount facing down |

## TVC & actuation

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 10 | TVC servos | MG90S metal-gear | 2 | TBD | TBD | pitch + yaw |
| 11 | Gimbaled nozzle | HoloModels 70mm TV nozzle STL, or Matzetas 360° scaled to 70mm | 1 | print | — | `cad/stl/tvc-nozzle-70mm.stl` |
| 12 | Servo bracket / gimbal mount | printed | 1 | print | — | `cad/stl/servo-bracket.stl` |
| 13 | Pushrods | 1.2 mm steel | 2 | TBD | TBD | + servo horns / clevises |
| 14 | PTFE dry lube | for ball-joint surfaces | 1 | TBD | TBD | sand joint first |

## Airframe & recovery

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 15 | Body tube | sized for 70mm EDF + duct | 1 | TBD | TBD | inlet area ≥ fan area (~38 cm²) |
| 16 | Nose cone | — | 1 | TBD | TBD | houses chute |
| 17 | Bulkheads / centering | — | as req'd | TBD | TBD | |
| 18 | Parachute | sized for ~1 kg, safe descent rate | 1 | TBD | TBD | |
| 19 | Shock cord + recovery hardware | — | 1 | TBD | TBD | |
| 20 | Recovery trigger | servo or MOSFET release | 1 | TBD | TBD | driven by GPIO 26 |

## Electrical / assembly

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 21 | 5V rail | perfboard strip / servo Y-harness / star | 1 | TBD | TBD | shared node, common ground |
| 22 | Hookup wire | silicone, appropriate gauge | — | TBD | TBD | fat path heavier gauge |
| 23 | Buzzer + status LED | 5V/3.3V | 1 ea | TBD | TBD | GPIO 19 / 20 |
| 24 | Battery divider resistors | e.g. 68k + 10k (ratio ~7.8) | 1 set | TBD | TBD | see wiring-diagram.md |
| 25 | Push button | latching/momentary, for arm | 1 | TBD | TBD | GPIO 2, INPUT_PULLUP |

---
**Gaps to fill (need from you):** purchase links/vendors and prices for each TBD
row, final airframe tube/nose-cone selection, and which items are reused from SPARC
vs newly purchased. Everything else (specs, quantities) comes from the design
summary and is ready.
