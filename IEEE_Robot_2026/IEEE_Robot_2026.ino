#include <Arduino.h>
#include <TinyStepper_28BYJ_48.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define MOTOR_1_IN1 A4
#define MOTOR_1_IN2 A3
#define MOTOR_1_IN3 A2
#define MOTOR_1_IN4 A1

#define STEPS_PER_REVOLUTION 2048

TinyStepper_28BYJ_48 stepper1;


void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off
  stepper1.connectToPins(MOTOR_1_IN1, MOTOR_1_IN2, MOTOR_1_IN3, MOTOR_1_IN4);
}

void loop() {
  if (Serial.available() >= 2) {  // command byte + data byte
    uint8_t cmd = Serial.read();
    uint8_t value = Serial.read();

    int8_t steps = (int8_t)value;

    switch (cmd) {
      case 0x01:  // Robotic Arm Stepper
        Serial.write(0xAA);
        stepperArm.step(steps);  //Positive = CW, Negative = CCW
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