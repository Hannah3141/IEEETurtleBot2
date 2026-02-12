#include <Arduino.h>
#include <Servo.h>

// Stepper Motor
#define ENA_1 2
#define STEP_1 3
#define DIR_1 4

// Servos
Servo leftServo;
Servo rightServo;
int leftPos = 0;
int rightPos = 0;

void setup() {
  pinMode(ENA_1, OUTPUT);
  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);

  leftServo.attach(8);
  rightServo.attach(9);
}

void loop() {
  motorstep(10, 1);
  turnServos(0);
  delay(5);
  motorstep(10, 0);
  turnServos(1);
}

void motorStep(int numSteps, int direction) {
  if (direction == 0) {
    digitalWrite(DIR_1, LOW);
  } else {
    digitalWrite(DIR_1, HIGH);
  }
  for (int x = 0; x < numSTEPS; x++) {
    digitalWrite(STEP_1, HIGH);
    delayMicroseconds(700);  // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(STEP_1, LOW);
    delayMicroseconds(700);
  }
}

void turnServos(int direction) {
  if (direction == 0) {
    rightPos = 180;
    for (leftPos = 0, leftPos < 180; leftPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      rightPos--;
      delay(15);
    }
  } else {
    for (rightPos = 0; rightPos < 180; rightPos++) {
      leftServo.write(leftPos);
      rightServo.write(rightPos);
      leftPos--;
      delay(15);
    }
  }
}