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
- **PID gains (yaw):** Kp __ Ki __ Kd __
- **TVC mechanical zero:** pitch __ µs · yaw __ µs
- **Deflection sign verified?** pitch ☐  yaw ☐  (must be ✓ before any free flight)
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
- [ ] TVC ±12° travel, no binding
- [ ] Telemetry received on ground laptop
- [ ] Recovery armed / staged (if applicable)

## Procedure
Step-by-step of what was actually done.

## Results
- Max altitude: __ m
- Max tilt: __ °
- Flight/run time: __ s
- Outcome: nominal / partial / abort / failure

## Observations
What happened, qualitatively. Sounds, oscillation, drift, recovery behavior.

## Anomalies & root cause
| Symptom | Suspected cause | Confirmed? |
|---|---|---|
| | | |

## Action items / next test
- [ ]
- [ ]
