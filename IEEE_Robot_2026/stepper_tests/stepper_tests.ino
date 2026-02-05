#define MOTOR_1_IN1 2
#define MOTOR_1_IN2 3
#define MOTOR_1_IN3 4
#define MOTOR_1_IN4 5

int stepPhase = 0;

void setup() {
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
  for (int i = 0; i < 100; i++) {
    motorstep(1);
    delay(5);
  }
}

void motorstep(int direction) {
  //
  // compute the next phase number
  //
  //stepPhase += (-direction);

  if (stepPhase <= -1)
    stepPhase = 3;

  if (stepPhase >= 4)
    stepPhase = 0;

  //
  // set the coils for this phase
  //
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
  stepPhase++;
}

void disableMotor() {
  digitalWrite(MOTOR_1_IN1, LOW);
  digitalWrite(MOTOR_1_IN2, LOW);
  digitalWrite(MOTOR_1_IN3, LOW);
  digitalWrite(MOTOR_1_IN4, LOW);
}