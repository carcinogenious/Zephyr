#include "battery.h"

#include <Arduino.h>

#include <zephyr_config.h>

namespace {
float volts_ = 0.0f;
battery::State state_ = battery::State::NOMINAL;
}  // namespace

void battery::init() {
    volts_ = 0.0f;
    state_ = State::NOMINAL;
}

void battery::update() {
    // Average a few mV reads to settle ADC noise, then scale by the divider ratio.
    uint32_t mv = 0;
    for (uint8_t i = 0; i < cfg::BATT_ADC_SAMPLES; i++) {
        mv += analogReadMilliVolts(cfg::PIN_BATT_ADC);
    }
    volts_ = (mv / (float)cfg::BATT_ADC_SAMPLES) / 1000.0f * cfg::BATT_DIVIDER_RATIO;

    // Latch downward only. CUT escalation wins regardless of the current state;
    // RAMP only ever sets from NOMINAL. A rebound above either threshold can never
    // clear an already-tripped state (that is the whole point of the latch).
    if (volts_ < cfg::BATT_FLOOR_V) {
        state_ = State::CUT;
    } else if (volts_ < cfg::BATT_RAMP_V && state_ == State::NOMINAL) {
        state_ = State::RAMP;
    }
}

float battery::volts() { return volts_; }

float battery::cellV() { return volts_ / cfg::BATT_CELLS; }

float battery::percent() {
    // Usable %: 0% at the protection floor (not a dead cell), 100% at full charge.
    float p = (volts_ - cfg::BATT_FLOOR_V) /
              (cfg::BATT_FULL_V - cfg::BATT_FLOOR_V) * 100.0f;
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return p;
}

bool battery::isLow() { return volts_ < cfg::BATT_RAMP_V; }

battery::State battery::state() { return state_; }

void battery::resetLatch() { state_ = State::NOMINAL; }
