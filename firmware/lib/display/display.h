// OLED display module — Zephyr EDF TVC rocket
// Ported from SPARC. Wraps the Heltec V3's on-board SSD1306 (128x64 @ 0x3C) on
// its dedicated I2C bus (Wire1 / pins SDA_OLED, SCL_OLED) so OLED traffic does
// not share the sensor bus on Wire. Each draw function calls display()
// internally — callers never touch the SSD1306Wire object directly.
#pragma once

#include <Arduino.h>

namespace display {

// Powers the OLED rail (Vext, GPIO 36 active-LOW), pulses the reset line
// (RST_OLED, GPIO 21), and brings up the SSD1306. Without the Vext enable the
// panel gets no power and every draw is a silent no-op — this is the "OLED
// won't turn on" failure carried over from SPARC.
void init();

// Generic centered two-line message for transient boot states (e.g. the
// barometer baseline capture: "Waiting for" / "pressure..."). Pass an empty
// second line for a single centered line.
void message(const String& line1, const String& line2 = String());

// Pre-launch sensor/battery readout for the pad state. One line per sensor, then
// the pack voltage + per-cell, and a final line with rough % charge — or a LOW
// warning when low is true.
void sensorStatus(bool bmpOk, bool mpuOk, bool tofOk,
                  float battV, float cellV, float pct, bool low);

// ARMED state: large "ARMED" banner with a countdown timer below.
void armed(int countdownSec);

// Post-landing flight summary: peak altitude (in), flight time (s),
// max tilt (deg).
void flightSummary(float peakAlt_in, float flightTime_s, float maxTilt_deg);

// Clears the screen. Called during LAUNCH..LANDED to free CPU cycles
// for the 50 Hz control loop.
void blank();

}  // namespace display
