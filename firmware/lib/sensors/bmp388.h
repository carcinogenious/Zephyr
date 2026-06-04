// BMP388 barometric altimeter driver — Zephyr EDF TVC rocket
// Ported from SPARC (same sensor, same Adafruit BMP3XX library). Reports
// altitude relative to the launch pad (AGL) in meters.
#pragma once

#include <Arduino.h>
#include <zephyr_config.h>

namespace bmp388 {

// Initializes the BMP388 on the shared I2C bus and confirms it responds.
// Call once after power-on. The launch zero is captured separately by
// captureBaseline(). Returns false if the sensor is not found.
bool init(uint8_t sdaPin = cfg::PIN_I2C_SDA, uint8_t sclPin = cfg::PIN_I2C_SCL);

// Captures the launch-pad altitude as the AGL zero. Samples until the reading
// stabilizes (spread under threshold) or timeoutMs elapses — hold the rocket
// still. Returns true if it stabilized; on timeout it still sets a best-effort
// baseline from the samples gathered and returns false. Call once after
// init(), before readAltitude() is relied on.
bool captureBaseline(unsigned long timeoutMs = 10000);

// Returns altitude relative to the launch site (AGL) in meters: the absolute
// pressure-altitude minus the baseline captured by captureBaseline().
// Returns NAN on an I2C read failure.
float readAltitude();

}  // namespace bmp388
