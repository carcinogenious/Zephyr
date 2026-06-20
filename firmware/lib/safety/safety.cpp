#include "safety.h"

#include <Arduino.h>
#include <math.h>

#include <zephyr_config.h>
#include <throttle.h>

namespace {
// Watchdog heartbeat: millis() at the last feed(). A 32-bit aligned scalar is
// read/written atomically on the ESP32, so the loop (core 1) and the monitor
// task (core 0) can share it without a lock.
volatile uint32_t lastFeedMs = 0;
volatile bool watchdogTripped = false;

// Sensor-fault timers: last time each critical sensor reported in. Touched only
// by check() on the control loop.
uint32_t lastImuLiveMs = 0;
uint32_t lastTofLiveMs = 0;

// Monitor task (core 0). If the loop (core 1) stops feeding the watchdog, cut
// the ESC to minimum and hold it cut — this is the last line of defence when the
// control loop itself has hung.
void watchdogTask(void*) {
    for (;;) {
        uint32_t since = millis() - lastFeedMs;
        if (since > cfg::SAFETY_LOOP_WATCHDOG_MS) {
            throttle::disarm();  // ESC → cfg::ESC_MIN_US, fan off
            if (!watchdogTripped) {
                watchdogTripped = true;
                Serial.print(F("WATCHDOG — loop stalled "));
                Serial.print(since);
                Serial.println(F(" ms, throttle cut"));
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
}  // namespace

void safety::init() {
    lastFeedMs = millis();
    watchdogTripped = false;
    xTaskCreatePinnedToCore(watchdogTask, "wdt", 2048, nullptr, 2, nullptr, 0);
}

void safety::feed() {
    lastFeedMs = millis();
}

safety::Trip safety::check(const Inputs& in) {
    uint32_t now = millis();

    // On the pad / not flying: keep the sensor-liveness timers current so a long
    // wait can't pre-age them, and never trip.
    if (!in.flying) {
        lastImuLiveMs = now;
        lastTofLiveMs = now;
        return Trip::NONE;
    }

    if (in.imuLive) lastImuLiveMs = now;
    if (in.tofLive) lastTofLiveMs = now;

    // 1. Tilt runaway. A NAN tilt (no attitude estimate) does NOT trip here — a
    //    dead IMU is caught by the sensor-fault rule below instead.
    if (!isnan(in.tilt_deg) && in.tilt_deg > cfg::SAFETY_MAX_TILT_DEG) {
        return Trip::TILT;
    }

    // 2. Flight ran past the hop-duration cap.
    if (in.flightTime_ms > cfg::FLIGHT_TIMEOUT_MS) {
        return Trip::FLIGHT_TIME;
    }

    // 3. A critical sensor stopped responding.
    if (now - lastImuLiveMs > cfg::SAFETY_SENSOR_TIMEOUT_MS ||
        now - lastTofLiveMs > cfg::SAFETY_SENSOR_TIMEOUT_MS) {
        return Trip::SENSOR_FAULT;
    }

    return Trip::NONE;
}

const char* safety::name(Trip t) {
    switch (t) {
        case Trip::TILT:         return "TILT";
        case Trip::FLIGHT_TIME:  return "FLIGHT_TIME";
        case Trip::SENSOR_FAULT: return "SENSOR_FAULT";
        default:                 return "NONE";
    }
}
