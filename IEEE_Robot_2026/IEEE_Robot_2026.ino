#include <Arduino.h>
#include <Servo.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define ENA_1 2
#define DIR_1 3
#define STEP_1 4

void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  // Setup Stepper Motors
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);
}

void loop() {
  if (Serial.available() >= 3) {  // command byte + 2 data bytes
    uint8_t cmd = Serial.read();
    uint8_t data1 = Serial.read();
    uint8_t data2 = Serial.read();

    switch (cmd) {
      case 0x01:  // Shovel Stepper
        Serial.write(0xAA);
        motorstep(data1, data2);  // 0 = down, 1 = up
        break;
      case 0x02:  // Relay
        if (data1 == 0x00) {
          digitalWrite(RELAY_PIN, LOW);  // OFF (active-low)
          Serial.write(0xAA);
        } else if (data1 == 0x01) {
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

void motorstep(int numSteps, int direction) {
  if (direction == 0) {  // move down
    digitalWrite(DIR_1, LOW);
  } else {  // move up
    digitalWrite(DIR_1, HIGH);
  }

  for (int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_1, HIGH);
    delay(1);
    digitalWrite(STEP_1, LOW);
    delay(1);
  }
}