#include <Arduino.h>
#include <Stepper.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define STEPS 100
Stepper stepperArm(STEPS, 8, 9, 10, 11);  // Stepper motor for robotic arm
#define ANALOG_ARM A1
#define TURN 10


void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off
  stepperArm.setSpeed(30);       // motor speed = 30RPMs
}

void loop() {
  if (Serial.available() >= 2) {  // command byte + data byte
    uint8_t cmd = Serial.read();
    uint8_t value = Serial.read();

    switch (cmd) {
      case 0x01:  // Relay
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
      case 0x02:              // Robotic Arm Stepper
        if (value == 0x00) {  // turn CW
          stepperArm.step(analogRead(ANALOG_ARM) + TURN);
        } else if (value == 0x01) {  // turn CCW
          stepperArm.step(analogRead(ANALOG_ARM) + 10);
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