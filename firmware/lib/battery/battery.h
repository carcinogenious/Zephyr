// battery.h — Zephyr 6S pack voltage sense + brownout protection policy
//
// Reads the divider on cfg::PIN_BATT_ADC every control loop (averaged to cut ADC
// noise), converts to real pack voltage, and classifies a LATCHED protection
// state. This module only SENSES and DECIDES — it never touches the ESC. The
// sequencer (lib/state_machine) reads state() and actuates: RAMP → a smooth
// powered throttle ramp-down (lib/throttle), CUT → a hard cut to minimum.
//
// Why latch: a sagged 6S pack rebounds when the fan unloads. Without a latch the
// recovered voltage would clear the trip and re-spin the fan — an oscillation.
#pragma once

#include <cstdint>

namespace battery {

enum class State : uint8_t {
    NOMINAL = 0,  // pack healthy
    RAMP,         // below cfg::BATT_RAMP_V  — ease throttle down under power
    CUT,          // below cfg::BATT_FLOOR_V — hard cut, fan off
};

// Call once in setup(). analogReadMilliVolts uses the eFuse ADC calibration, so
// there is nothing to configure — this just clears the cached reading + state.
void init();

// Sample + average cfg::BATT_ADC_SAMPLES reads, convert to pack volts, and update
// the latched protection state. Call every control loop (also in flight).
void update();

float volts();    // pack voltage (V), last update()
float cellV();    // per-cell voltage (V)
float percent();  // rough state-of-charge, 0..100 (linear FLOOR..FULL)
bool  isLow();    // instantaneous volts < cfg::BATT_RAMP_V (for the display warning)

State state();       // LATCHED protection state (for actuation)
void  resetLatch();  // re-evaluate from NOMINAL — call on a fresh arm

}  // namespace battery
