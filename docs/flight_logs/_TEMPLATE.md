<!--
Flight / test log template. Copy this file to YYYY-MM-DD-<name>.md for each
test or flight (e.g. 2026-07-14-esc-bench.md, 2026-07-20-hop1.md). One file per
event. Keep it factual — this is the record you tune and debug against later.
-->

# <YYYY-MM-DD> — <event name>

| | |
|---|---|
| **Date / time** | |
| **Type** | bench / test-stand / tethered / free-hop / flight |
| **Location** | |
| **Crew** | |
| **Firmware** | git commit / branch |
| **Telemetry log** | `data/flights/<file>.csv` |

## Objective
What this test is meant to prove. One or two sentences.

## Configuration
- **Throttle profile:** open-loop ramp / closed-loop · launch throttle = __%
- **PID gains (pitch):** Kp __ Ki __ Kd __
- **PID gains (roll):** Kp __ Ki __ Kd __
- **TVC mechanical zero:** pitch __ µs · roll __ µs
- **Deflection sign verified?** pitch ☐  roll ☐  (must be ✓ before any free flight)
- **Mass (AUW):** __ g
- **Other changes since last log:**

## Conditions
- Wind: __ · Temp: __ · Surface/clamp:
- Battery: __ V start · __ mAh / __ % used

## Pre-flight checklist
Reference build-guide Part D. Note any deviations.
- [ ] Common ground verified
- [ ] Fan clamped (for any ESC/throttle bench test)
- [ ] All 3 sensors reading correctly
- [ ] ESC armed (beeps confirmed)
- [ ] TVC full ±20° mechanical travel, no binding (firmware clamps to TVC_CLAMP_DEG)
- [ ] Telemetry received on ground laptop
- [ ] Landing zone clear & marked (propulsive touchdown — no chute)

## Procedure
Step-by-step of what was actually done.

## Results
- Outcome: nominal / partial / abort / failure

## Telemetry summary
Paste the block from the WiFi control page (shown after landing) here — it is the
auto-generated end-of-flight summary:

<!-- paste from the "Flight summary" box at http://192.168.4.1 -->
- Peak altitude: __ in
- Max tilt: __ deg
- Flight time: __ s

## Observations
What happened, qualitatively. Sounds, oscillation, drift, descent + touchdown behavior.

## Anomalies & root cause
| Symptom | Suspected cause | Confirmed? |
|---|---|---|
| | | |

## Action items / next test
- [ ]
- [ ]
