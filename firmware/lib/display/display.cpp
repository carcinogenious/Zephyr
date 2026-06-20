#include "display.h"

#include <SSD1306Wire.h>

namespace {
// I2C_TWO routes the OLED onto Wire1 so it does not clobber the sensor
// bus on Wire (which the BMP388/MPU6050/VL53L1X drivers init on pins
// 41/42). The Heltec V3 OLED is wired to SDA_OLED/SCL_OLED.
SSD1306Wire oled(0x3C, SDA_OLED, SCL_OLED, GEOMETRY_128_64, I2C_TWO);
}  // namespace

void display::init() {
    // Heltec V3: the on-board OLED is powered from the Vext rail, gated by
    // GPIO 36 (active LOW — driving it LOW switches the rail ON). Without
    // this the panel gets no power and every draw is a silent no-op, even
    // though the SSD1306 init and RST pulse below run without error. This is
    // the "OLED won't turn on" issue from SPARC. Give the rail a moment to
    // settle before resetting the controller.
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(50);

    // Heltec V3: the SSD1306 RST line is on GPIO 21 and must be pulsed
    // low-then-high before init() or the controller stays in reset and
    // every subsequent draw is a no-op.
    pinMode(RST_OLED, OUTPUT);
    digitalWrite(RST_OLED, HIGH);
    delay(20);
    digitalWrite(RST_OLED, LOW);
    delay(20);
    digitalWrite(RST_OLED, HIGH);

    oled.init();
    oled.flipScreenVertically();
    oled.clear();
    oled.display();
}

void display::message(const String& line1, const String& line2) {
    oled.clear();
    oled.setFont(ArialMT_Plain_16);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(64, 14, line1);
    if (line2.length() > 0) {
        oled.drawString(64, 34, line2);
    }
    // Restore the default the other screens draw with (drawString at x=0).
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.display();
}

void display::sensorStatus(bool bmpOk, bool mpuOk, bool tofOk,
                           float battV, float cellV, float pct, bool low) {
    oled.clear();
    oled.setFont(ArialMT_Plain_10);
    oled.drawString(0, 0,  String("BMP388  ") + (bmpOk ? "OK" : "FAIL"));
    oled.drawString(0, 11, String("MPU6050 ") + (mpuOk ? "OK" : "FAIL"));
    oled.drawString(0, 22, String("VL53L1X ") + (tofOk ? "OK" : "FAIL"));
    oled.drawString(0, 33, String("Batt ") + String(battV, 1) + "V  "
                                           + String(cellV, 2) + "/c");
    oled.drawString(0, 44, low ? String("** LOW BATTERY **")
                               : String("SOC  ") + String(pct, 0) + "%");
    oled.display();
}

void display::armed(int countdownSec) {
    oled.clear();
    oled.setFont(ArialMT_Plain_24);
    oled.drawString(0, 0, "ARMED");
    oled.setFont(ArialMT_Plain_16);
    oled.drawString(0, 36, String("T-") + countdownSec + "s");
    oled.display();
}

void display::flightSummary(float peakAlt_in, float flightTime_s,
                            float maxTilt_deg) {
    oled.clear();
    oled.setFont(ArialMT_Plain_10);
    oled.drawString(0, 0,  String("Peak  ") + String(peakAlt_in, 1) + " in");
    oled.drawString(0, 11, String("Time  ") + String(flightTime_s, 1) + " s");
    oled.drawString(0, 22, String("Tilt  ") + String(maxTilt_deg, 1) + " deg");
    oled.display();
}

void display::blank() {
    oled.clear();
    oled.display();
}
