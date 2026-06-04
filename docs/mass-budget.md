# Zephyr — Mass Budget & Thrust/Weight

Tracks component mass and the resulting thrust-to-weight (T/W) margins.

**Status:** pre-build estimate. Only the EDF and battery masses are locked (from
the design summary); everything else is a spec-sheet estimate or a back-calculated
budget. **Replace estimates with weighed values as parts arrive** — a mass budget
is only useful once it reflects reality.

`Source` legend: **locked** = fixed design decision · **est.** = spec-sheet /
typical, verify by weighing · **budget** = remainder backed out of the all-up target.

## Component masses

| Component | Mass (g) | Source | Notes |
|---|---:|---|---|
| Powerfun 70mm 6S EDF (motor + fan + housing) | 178 | locked | |
| OVONIC 6S 1300mAh 120C LiPo | 232 | locked | heaviest single item |
| ESC (80A, 2–6S, w/ BEC) | 70 | est. | varies by brand/wire length |
| Heltec WiFi LoRa 32 V3 | 15 | est. | with headers/antenna |
| BMP388 breakout | 1.3 | est. | |
| MPU6050 (GY-521) | 3 | est. | |
| VL53L1X (TOF400C) | 2.5 | est. | |
| TVC servo — MG90S #1 (pitch) | 13.4 | est. | metal-gear MG90S |
| TVC servo — MG90S #2 (yaw) | 13.4 | est. | |
| Gimbaled nozzle (printed) | 40 | est. | depends on material/infill |
| Servo bracket / gimbal mount (printed) | 25 | est. | |
| Pushrods + linkage hardware | 5 | est. | 1.2 mm steel + horns |
| Wiring, XT60s, 5V rail, connectors | 30 | est. | |
| Parachute + shock cord + recovery hw | 30 | est. | sized for ~1 kg |
| **Subtotal (above)** | **~659** | | |
| Airframe (tube, nose cone, bulkheads, intake ring, fins) | ~326 | budget | remainder to target |
| **All-up weight (AUW) target** | **~985** | locked | design range 970–1000 g |

The airframe line is a **budget, not a measurement**: it's whatever's left between
the components and the ~985 g target. If the structure comes in over ~326 g, AUW
rises and the T/W margins below shrink — track it.

## Thrust / weight

| Quantity | Value |
|---|---|
| Max static thrust (bench) | 2240 g-f |
| Effective thrust (−15% intake loss) | ~1904 g-f |
| AUW | ~985 g |
| **T/W (bench)** | 2240 / 985 = **2.27** |
| **T/W (real, intake loss)** | 1904 / 985 = **1.93** |

- **Hover throttle ≈ 45%** (locked design figure). Note EDF thrust is *not* linear
  with throttle, so 45% throttle ≠ 45% thrust — treat this as the expected setting
  to validate on the ESC bench test, not a derived number.
- **Control-margin guardrail:** to keep real-thrust T/W ≥ 1.5 (healthy authority
  for TVC), AUW must stay below 1904 / 1.5 ≈ **1270 g**. At ~985 g there's room,
  but it erodes fast if the airframe or wiring run heavy.

## How to use this doc

1. Weigh each component as it arrives; replace the `est.` value and flip Source to
   a measured marker.
2. Re-weigh the finished airframe and replace the `budget` line with the actual.
3. Recompute the T/W rows from the new AUW.
4. If real-thrust T/W drops below ~1.5, cut mass before flying.

---
**Gaps to fill (need from build):** measured masses for the ESC, Heltec, sensors,
servos, printed parts, parachute, and the finished airframe. The two locked values
(EDF 178 g, battery 232 g) and the thrust figures come from the design summary.
