#include <Arduino.h>
#include <Servo.h>

// Relay for Electromagnet
#define RELAY_PIN A6

//Stepper Motors
#define MOTOR_1_IN1 2
#define MOTOR_1_IN2 3
#define MOTOR_1_IN3 4
#define MOTOR_1_IN4 5
int stepPhase = 0;

//Servos
Servo leftServo;
Servo rightServo;
int pos = 0;

void setup() {
  Serial.begin(115200);  // USB serial to Pi
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // relay off

  //setup servo
  leftServo.attach(9);

  // setup Motor 1
  pinMode(MOTOR_1_IN1, OUTPUT);
  digitalWrite(MOTOR_1_IN1, LOW);

  pinMode(MOTOR_1_IN2, OUTPUT);
  digitalWrite(MOTOR_1_IN2, LOW);

  pinMode(MOTOR_1_IN3, OUTPUT);
  digitalWrite(MOTOR_1_IN3, LOW);

  pinMode(MOTOR_1_IN4, OUTPUT);
  digitalWrite(MOTOR_1_IN4, LOW);
}

void loop() {
  if (Serial.available() >= 3) {  // command byte + 2 data bytes
    uint8_t cmd = Serial.read();
    uint8_t data1 = Serial.read();
    uint8_t data2 = Serial.read();

    switch (cmd) {
      case 0x01:  // Robotic Arm Stepper
        Serial.write(0xAA);
        motorstep(data1, data2);  //Positive = CW, Negative = CCW
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
      case 0x03:  // Servos
        if (data1 == 0x00) {
          for (pos = 0; pos <= 180; pos++) {
            leftServo.write(pos);  // tell servo to go to position
            rightServo.write(pos);
            delay(15);  // waits 15 ms for the servo to reach the position
          }
        }
        if (data1 == 0x01) {
          for (pos = 180; pos >= 0; p--) {
            leftServo.write(pos);
            rightServo.write(pos);
            delay(15);
          }
        }
        break;
      default:
        Serial.write(0xFF);  // ERROR: unknown command
        break;
    }
  }
}

void motorstep(int numSteps, int direction) {
  for (int i = 0; i < numSteps; i++) {
    if (stepPhase <= -1)
      stepPhase = 3;

    if (stepPhase >= 4)
      stepPhase = 0;

    switch (stepPhase) {
      case 0:
        digitalWrite(MOTOR_1_IN1, LOW);
        digitalWrite(MOTOR_1_IN2, LOW);
        digitalWrite(MOTOR_1_IN3, HIGH);
        digitalWrite(MOTOR_1_IN4, HIGH);
        break;
      case 1:
        digitalWrite(MOTOR_1_IN1, LOW);
        digitalWrite(MOTOR_1_IN2, HIGH);
        digitalWrite(MOTOR_1_IN3, HIGH);
        digitalWrite(MOTOR_1_IN4, LOW);
        break;
      case 2:
        digitalWrite(MOTOR_1_IN1, HIGH);
        digitalWrite(MOTOR_1_IN2, HIGH);
        digitalWrite(MOTOR_1_IN3, LOW);
        digitalWrite(MOTOR_1_IN4, LOW);
        break;
      case 3:
        digitalWrite(MOTOR_1_IN1, HIGH);
        digitalWrite(MOTOR_1_IN2, LOW);
        digitalWrite(MOTOR_1_IN3, LOW);
        digitalWrite(MOTOR_1_IN4, HIGH);
        break;
    }
    if (direction == 0) {
      stepPhase--;
    } else {
      stepPhase++;
    }
    delay(5);
  }
}