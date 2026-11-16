#include <Wire.h>

const int PIN_SDA = 8;
const int PIN_SCL = 9;
const int PIN_FL_PWM = 4;
const int PIN_FR_PWM = 5;

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  pinMode(PIN_FL_PWM, OUTPUT);
  pinMode(PIN_FR_PWM, OUTPUT);
  Serial.println("ESP32-S3 Outrunner Buggy Controller Ready");
}

void loop() {
  // Simulate SimpleFOC 3-phase PWM duty cycle ramp
  for (int duty = 40; duty <= 200; duty += 40) {
    analogWrite(PIN_FL_PWM, duty);
    analogWrite(PIN_FR_PWM, duty);
    Serial.printf("Drive PWM Duty: %d/255\n", duty);
    delay(300);
  }
}
