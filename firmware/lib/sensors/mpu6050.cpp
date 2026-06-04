#include "mpu6050.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

namespace {
Adafruit_MPU6050 mpu;
float gyroBias_x = 0.0f;
float gyroBias_y = 0.0f;
float gyroBias_z = 0.0f;
}  // namespace

bool mpu6050::init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);

    if (!mpu.begin(cfg::ADDR_MPU6050, &Wire)) {
        return false;
    }

    // Match the spec from CLAUDE.md: ±8 g accel, ±2000 °/s gyro.
    // 21 Hz DLPF gives anti-aliasing margin under the 50 Hz control loop.
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Capture gyro static bias while stationary. Without this, even a small
    // resting offset (~0.01 rad/s) integrates into several degrees of
    // attitude drift over a multi-second flight.
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    int samples = 0;
    sensors_event_t a, g, temp;
    for (int i = 0; i < 200; i++) {
        if (mpu.getEvent(&a, &g, &temp)) {
            sx += g.gyro.x;
            sy += g.gyro.y;
            sz += g.gyro.z;
            samples++;
        }
        delay(5);
    }
    if (samples == 0) {
        return false;
    }
    gyroBias_x = sx / samples;
    gyroBias_y = sy / samples;
    gyroBias_z = sz / samples;
    return true;
}

mpu6050::Reading mpu6050::read() {
    Reading r{};
    sensors_event_t a, g, temp;
    if (!mpu.getEvent(&a, &g, &temp)) {
        r.ok = false;
        return r;
    }
    r.accel_x = a.acceleration.x;
    r.accel_y = a.acceleration.y;
    r.accel_z = a.acceleration.z;
    r.gyro_x = g.gyro.x - gyroBias_x;
    r.gyro_y = g.gyro.y - gyroBias_y;
    r.gyro_z = g.gyro.z - gyroBias_z;
    r.ok = true;
    return r;
}
