#include <Arduino.h>
#include <Stepper.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define STEPNUM 1000
Stepper stepperArm(STEPNUM, A1, A2, A3, A4);  // Stepper motor for robotic arm

void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off
  stepperArm.setSpeed(30);       // motor speed = 30RPMs
}

void loop() {
  if (Serial.available() >= 2) {  // command byte + data byte
    int8_t cmd = Serial.read();
    int8_t value = Serial.read();

    switch (cmd) {
      case 0x01:  // Robotic Arm Stepper
        Serial.write(values);
        stepperArm.step(values);  //Positive = CW, Negative = CCW
        Serial.write(0xAA);
        break;
      case 0x02:  // Relay
        if (value == 0x00) {
          digitalWrite(RELAY_PIN, LOW);  // OFF (active-low)
          Serial.write(0xAA);
        } else if (value == 0x01) {
          digitalWrite(RELAY_PIN, HIGH);  // ON (active-low)
          Serial.write(0xAA);
        } else {
          Serial.write(0xFF);
        }
        break;
      default:
        Serial.write(0xFF);  // ERROR: unknown command
        break;
    }
  }
}