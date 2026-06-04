# Zephyr — Wiring Diagram

Power rail, pin map, and grounding for the EDF TVC rocket. Pin numbers are the
authoritative copy in [`firmware/include/config.h`](../firmware/include/config.h);
this document explains how they connect. See
[`build-guide.md`](build-guide.md) Part A Steps 2–3 for the bench procedure.

> **#1 first-build failure is a floating ground.** Every ground in the system —
> battery, ESC, ESP32, servos, sensors — ties to one common GND node.

## Power architecture

One 6S battery. Two paths leave the ESC: the high-current "fat path" to the fan,
and the BEC's regulated 5V to everything logic-side. Sensors run off the ESP32's
own 3.3V regulator.

```
                 ┌──────────────────────── 6S LiPo (22.2V nom, 25.2V max) ──────────┐
                 │  +                                                              − │
                 │  └── XT60 ──┐                                          XT60 ──┐    │
                 ▼            ▼                                                  ▼
            (balance)    ┌─────────────── ESC (80A, 2–6S) ───────────────┐
            charger      │  Vin+ 22.2V          3 phase ──▶ EDF MOTOR     │
                         │  Vin−  ───────┐      signal ◀── GPIO 47 (ESP32)│
                         │  BEC +5V/5A ──┼──┐                             │
                         │  BEC GND ─────┤  │                             │
                         └───────────────┘  │                             │
                                         │  │
              ┌──────────────────────────┘  └────── +5V NODE ──────────────────────┐
              ▼ GND NODE (star)                       │                              │
   ┌──────────┼──────────┬───────────┬──────┐        ├── ESP32  5V pin              │
   │          │          │           │      │        ├── Servo PITCH (+)  (MG90S)   │
 batt−      ESC−     servo −'s    sensor    ESP32     ├── Servo YAW   (+)  (MG90S)   │
 (via XT60) (BEC−)   (P/Y/recov)   GND      GND       └── Recovery trigger (+)       │
                                                                                     │
   ESP32 3.3V regulator ──▶ 3.3V to BMP388 / MPU6050 / VL53L1X (+ OLED via BSP) ─────┘
```

**BEC current budget:** 5V / 5A available. ESP32 ~0.3A + 2× MG90S ~1.4A peak
≈ 1.7A. Recovery actuator is momentary. Comfortable margin.

**Build the 5V rail as a shared node, not a daisy-chain** (perfboard strip,
soldered star, or a servo Y-harness): BEC +5V → +5V node → branch to each load.

## Pin map (Heltec WiFi LoRa 32 V3, ESP32-S3)

| Pin | Net | Connects to | Notes |
|-----|-----|-------------|-------|
| GPIO 41 | I2C SDA | BMP388 + MPU6050 + VL53L1X | shared bus, 400 kHz |
| GPIO 42 | I2C SCL | same three sensors | |
| GPIO 47 | ESC signal | ESC throttle wire | 1000–2000 µs, ESP32Servo |
| GPIO 48 | Servo PWM | TVC pitch servo (MG90S) | 1000–2000 µs |
| GPIO 3  | Servo PWM | TVC yaw servo (MG90S) | 1000–2000 µs |
| GPIO 26 | Output | Recovery deploy (servo / MOSFET gate) | spare-capable pin |
| GPIO 2  | Input | Arm/launch button | INPUT_PULLUP, active-low |
| GPIO 1  | ADC | 6S battery voltage divider | see divider below |
| GPIO 19 | Output | Buzzer | |
| GPIO 20 | Output | Status LED | |
| —       | I2C (Wire1) | On-board OLED SSD1306 (0x3C) | BSP pins SDA_OLED/SCL_OLED/RST_OLED |

**Heltec V3 pin cautions:** GPIO 6/7 = module SPI flash (never use); GPIO 8–14 =
on-board SX1262 LoRa SPI (radio disabled); GPIO 26–32 = SPI-flash interface (26
usable as spare only — no PSRAM on this module); GPIO 0/45/46 = strapping/boot
pins. Do not reassign the OLED BSP pins.

## I2C bus

All three sensors and the OLED share I2C. The sensors are on the main bus
(GPIO 41/42); the OLED is on the Heltec's second bus via BSP macros.

| Device | Address | Bus |
|--------|---------|-----|
| BMP388 (baro) | 0x76 (0x77 if SDO high) | Wire (41/42) |
| MPU6050 (IMU) | 0x68 (0x69 if AD0 high) | Wire (41/42) |
| VL53L1X (ToF) | 0x29 | Wire (41/42) |
| SSD1306 (OLED) | 0x3C | Wire1 (BSP) |

Wire each sensor: **SDA, SCL, 3.3V, GND** from the ESP32. Confirm all three
appear on an I2C scan before proceeding (build-guide Part A Step 1).

### OLED won't turn on (Heltec V3) — fix

The on-board SSD1306 is powered from the **Vext rail, gated by GPIO 36
(`Vext`), active-LOW**. You must drive `Vext` LOW to switch the panel's power
on, *then* pulse the reset line `RST_OLED` (GPIO 21) low→high, before calling
`oled.init()`. Skip the Vext enable and the SSD1306 init and every draw run
without error but nothing lights up — a silent no-op. This is handled in
`firmware/lib/display/display.cpp::init()` (carried over from SPARC, which hit
the same issue). Do not reassign GPIO 36 or 21.

## Battery voltage sense (6S)

A resistor divider scales pack voltage (25.2 V max) into the ESP32's 0–3.3 V ADC
range on GPIO 1.

```
  Pack+ ──[ R1 ]──┬──[ R2 ]── GND
                  │
                  └──▶ GPIO 1 (ADC)
```

- `V_adc = V_pack × R2 / (R1 + R2)`; divider ratio `(R1+R2)/R2` is
  `BATT_DIVIDER_RATIO` in `config.h` (**default 8.0 — CALIBRATE against a meter**).
- Pick R1/R2 so V_adc stays ≤ 3.3 V at 25.2 V (e.g. R1 = 68k, R2 = 10k → ratio
  7.8, max V_adc ≈ 3.23 V). Use a small filter cap if the reading is noisy.
- Cutoffs in `config.h`: warn 3.50 V/cell (21.0 V pack), critical 3.30 V/cell
  (19.8 V pack).

## Grounding (read this twice)

Tie ALL grounds to one common node:
- Battery − (through XT60)
- ESC power − and BEC −
- ESP32 GND
- Both TVC servo −, recovery actuator −
- Sensor GND, OLED GND (via the ESP32)

A shared ground reference is what makes the PWM signals valid. A floating or
daisy-chained ground produces jittery or dead servos/ESC — the most common
first-build failure.
