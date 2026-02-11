#define STEP_1 2
#define DIR_1 3

void setup() {
  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
}

void loop() {
  motorstep(10, 1);
  delay(5);
  motorstep(10, 0);
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