#include <Arduino.h>
#include <Servo.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define ENA_1 2
#define DIR_1 3
#define STEP_1 4

// Servos
Servo leftServo;
Servo rightServo;
int leftPos = 0;
int rightPos = 0;

void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  // Setup Stepper Motors
  pinMode(ENA_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  digitalWrite(ENA_1, LOW);

  leftServo.attach(8);
  rightServo.attach(9);
}

void loop() {
  if (Serial.available() >= 3) {  // command byte + 2 data bytes
    uint8_t cmd = Serial.read();
    uint8_t data1 = Serial.read();
    uint8_t data2 = Serial.read();

    switch (cmd) {
      case 0x01:  // Shovel Stepper
        motorstep(data1, data2);  // 0 = down, 1 = up
        Serial.write(0xAA);
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
      case 0x03: // Servos
        turnServos(data1);
        Serial.write(0xAA);
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

void turnServos(int direction) {
  if (direction == 0) {
    rightPos = 180;
    for (leftPos = 0; leftPos < 180; leftPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      rightPos--;
      delay(15);
    }
  } else {
    leftPos = 180;
    for (rightPos = 0; rightPos < 180; rightPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      leftPos--;
      delay(15);
    }
  }
}