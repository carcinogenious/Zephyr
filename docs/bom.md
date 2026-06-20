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
| 10 | TVC servos | MG90S metal-gear | 2 | TBD | TBD | pitch + roll |
| 11 | Gimbaled nozzle | HoloModels 70mm TV nozzle STL, or Matzetas 360° scaled to 70mm | 1 | print | — | `cad/stl/tvc-nozzle-70mm.stl` |
| 12 | Servo bracket / gimbal mount | printed | 1 | print | — | `cad/stl/servo-bracket.stl` |
| 13 | Pushrods | 1.2 mm steel | 2 | TBD | TBD | + servo horns / clevises |
| 14 | PTFE dry lube | for ball-joint surfaces | 1 | TBD | TBD | sand joint first |

## Airframe

Landing is propulsive — no parachute, shock cord, or deploy actuator. The lower
airframe / legs absorb the low-speed powered touchdown instead.

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 15 | Body tube | sized for 70mm EDF + duct | 1 | TBD | TBD | inlet area ≥ fan area (~38 cm²) |
| 16 | Nose cone | — | 1 | TBD | TBD | aerodynamic; no chute bay |
| 17 | Bulkheads / centering | — | as req'd | TBD | TBD | |
| 18 | Landing legs / skid | absorbs powered touchdown | 1 set | TBD | TBD | propulsive landing |

## Electrical / assembly

| # | Item | Spec | Qty | Source | Price | Notes |
|---|---|---|---:|---|---:|---|
| 19 | 5V rail | perfboard strip / servo Y-harness / star | 1 | TBD | TBD | shared node, common ground |
| 20 | Hookup wire | silicone, appropriate gauge | — | TBD | TBD | fat path heavier gauge |
| 21 | Battery divider resistors | 100k + 10k (ratio 11) | 1 set | TBD | TBD | 6S sense on GPIO 1; see wiring-diagram.md |
| 22 | ESC signal level shifter | 74AHCT125N (MUST be AHCT/HCT) | 1 | TBD | TBD | 3.3→5V buffer so the ESC arms; see wiring-diagram.md |
| 23 | Pulldown resistor | 10k | 1 | TBD | TBD | GPIO 6 → GND; holds buffer input low at boot |

---
**Gaps to fill (need from you):** purchase links/vendors and prices for each TBD
row, final airframe tube/nose-cone selection, and which items are reused from SPARC
vs newly purchased. Everything else (specs, quantities) comes from the design
summary and is ready.
