# Zephyr — Wiring Diagram

Power rail, pin map, and grounding for the EDF TVC rocket. Pin numbers are the
authoritative copy in [`firmware/include/zephyr_config.h`](../firmware/include/zephyr_config.h);
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
                    │  Vin−  ───────┐      signal ◀── 5V PWM (buf 1Y)│
                    │  BEC +5V/5A ──┼──┐            ▲                 │
                    │  BEC GND ─────┤  │            │                 │
                    └───────────────┘  │   ┌────────┴──────────┐      │
                                    │  │   │ 74AHCT125N buffer  │ ◀── GPIO 6 (3.3V PWM → 1A)
                                    │  │   │ 3.3V → 5V          │     +10k pulldown GPIO 6→GND
         ┌──────────────────────────┘  │   └ VCC←+5V  GND→star ┘     1OE(pin 1)→GND
         ▼ GND NODE (star)             │
  ┌──────┼──────┬───────┬──────┬──────┬┴──────┐    ┌──── +5V NODE ──────────────────┐
  │      │      │       │      │      │       │    ├── ESP32  5V pin                 │
 batt−  ESC−  servo−'s sensor ESP32  buf GND  │    ├── Servo PITCH (+)  (MG90S)      │
 (XT60) (BEC−)(P/R)    GND    GND   (pin 7)   │    ├── Servo ROLL  (+)  (MG90S)      │
                                              │    └── 74AHCT125N VCC (pin 14)       │
  ESP32 3.3V regulator ──▶ 3.3V to BMP388 / MPU6050 / VL53L1X (+ OLED via BSP)
```

The ESC ignores a 3.3V signal (it needs ~5V to arm), so the GPIO 6 PWM passes
through a **74AHCT125N** buffer that translates 3.3V → 5V on the wire. The buffer
is transparent to firmware — native LEDC (ch0) drives GPIO 6 at 1000–2000µs / 50Hz.
See the level-shifter table below for the full pinout.

**BEC current budget:** 5V / 5A available. ESP32 ~0.3A + 2× MG90S ~1.4A peak +
74AHCT125N (negligible, ~µA) ≈ 1.7A. Comfortable margin.

**Build the 5V rail as a shared node, not a daisy-chain** (perfboard strip,
soldered star, or a servo Y-harness): BEC +5V → +5V node → branch to each load.

## Pin map (Heltec WiFi LoRa 32 V3, ESP32-S3)

| Pin | Net | Connects to | Notes |
|-----|-----|-------------|-------|
| GPIO 41 | I2C SDA | BMP388 + MPU6050 + VL53L1X | shared bus, 400 kHz |
| GPIO 42 | I2C SCL | same three sensors | |
| GPIO 6  | ESC signal | ESC throttle wire **via 74AHCT125N** | 1000–2000 µs, LEDC ch0; buffered 3.3→5V; 10k pulldown to GND |
| GPIO 48 | Servo PWM | TVC pitch servo (MG90S) | 1000–2000 µs |
| GPIO 5  | Servo PWM | TVC roll servo (MG90S) | 1000–2000 µs (not GPIO 3 — strapping pin) |
| GPIO 26 | —      | Free spare (no parachute — propulsive landing) | usable as a spare only |
| GPIO 1  | ADC | 6S battery voltage divider | see divider below |
| GPIO 2 / 19 / 20 | — | Free spares (no arm button / buzzer / LED) | arm/launch/abort + status via WiFi page |
| —       | I2C (Wire1) | On-board OLED SSD1306 (0x3C) | BSP pins SDA_OLED/SCL_OLED/RST_OLED |

**Heltec V3 (ESP32-S3FN8) pin cautions:** GPIO 8–14 = on-board SX1262 LoRa SPI
(radio disabled); GPIO 26–32 = module SPI-flash interface (26 usable as a spare
only — no PSRAM on this module); GPIO 0/3/45/46 = strapping/boot pins (GPIO 3 is
JTAG-select on the S3 — unsuitable for servo/sensor; roll is on GPIO 5). GPIO 47/48
sit next to the USB-UART lines and pick up noise on continuous PWM — the ESC was
moved off 47 to **GPIO 6** for this reason (GPIO 6/7 are ordinary free GPIOs on
the S3, *not* SPI-flash pins as on the classic ESP32); GPIO 48 still drives the
pitch servo acceptably. Avoid GPIO 4 (demonstrated wiring fault). Do not reassign
the OLED BSP pins.

## ESC level shifter (74AHCT125N)

The ESP32-S3 PWM is only 3.3V; the ESC needs ~5V to register and arm (confirmed: a
5V Arduino Nano Every armed the same ESC, the 3.3V ESP32 did not). One channel of a
**74AHCT125N** buffers GPIO 6 up to a clean 5V PWM. It **must be AHCT/HCT** — the
"T" gives it TTL input thresholds, so it reads 3.3V as logic HIGH while powered at
5V. A plain 74AC125 / 74HC125 will *not* work.

| 74AHCT125N pin | Connects to |
|----------------|-------------|
| 14 (VCC) | +5V rail (from ESC BEC) |
| 7 (GND) | common ground (star) |
| 1 (1OE, active-low) | GND (enables the output) |
| 2 (1A, input) | GPIO 6 (3.3V PWM from ESP32) |
| 3 (1Y, output) | ESC signal wire (5V PWM out) |
| 4,5,6,8–13 | unused |

Add a **10k pull-down from GPIO 6 to GND** so the buffer input is defined LOW
while the ESP32 boots (pins float briefly), keeping the ESC at a clean minimum
from power-up. The 74AHCT125N GND, ESP32 GND, and ESC/BEC GND all share the one
common node — see Grounding below.

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

- Built + verified: **R1 = 100k, R2 = 10k** → ratio `(R1+R2)/R2` = **11.0**, so
  `V_adc = V_pack / 11`. Measured 22.8 V pack → 2.07 V at the junction; 25.2 V max
  → 2.29 V (≤ 3.3 V ✓). Use a small filter cap if the reading is noisy.
- Firmware reads it with `analogReadMilliVolts` (eFuse-calibrated mV) and scales by
  `BATT_DIVIDER_RATIO` in `zephyr_config.h` (**11.0 — the single CALIBRATE knob;
  trim until the reported pack voltage matches a multimeter**).
- Per-cell protection in `zephyr_config.h` (×6 → pack): ramp-down begins at
  3.40 V/cell (20.4 V), hard cut at 3.30 V/cell (19.8 V); 4.20 V/cell (25.2 V) is
  the full-charge reference for the % display. Both trips LATCH — see `lib/battery`.

## Grounding (read this twice)

Tie ALL grounds to one common node:
- Battery − (through XT60)
- ESC power − and BEC −
- ESP32 GND
- Both TVC servo −
- 74AHCT125N GND (pin 7) — and its 1OE (pin 1) tied to GND
- Sensor GND, OLED GND (via the ESP32)

A shared ground reference is what makes the PWM signals valid. A floating or
daisy-chained ground produces jittery or dead servos/ESC — the most common
first-build failure.
