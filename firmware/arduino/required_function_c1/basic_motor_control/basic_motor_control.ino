#include <Arduino.h>

// Motor pin mapping copied from Car_Volt_Feedback_24A.ino
#define PWMA 12
#define DIRA1 34
#define DIRA2 35

#define PWMB 8
#define DIRB1 37
#define DIRB2 36

#define PWMC 9
#define DIRC1 43
#define DIRC2 42

#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

#define MOTORA_FORWARD(pwm) do { digitalWrite(DIRA1, LOW);  digitalWrite(DIRA2, HIGH); analogWrite(PWMA, pwm); } while (0)
#define MOTORA_BACKOFF(pwm) do { digitalWrite(DIRA1, HIGH); digitalWrite(DIRA2, LOW);  analogWrite(PWMA, pwm); } while (0)
#define MOTORA_STOP()       do { digitalWrite(DIRA1, LOW);  digitalWrite(DIRA2, LOW);  analogWrite(PWMA, 0); } while (0)

#define MOTORB_FORWARD(pwm) do { digitalWrite(DIRB1, LOW);  digitalWrite(DIRB2, HIGH); analogWrite(PWMB, pwm); } while (0)
#define MOTORB_BACKOFF(pwm) do { digitalWrite(DIRB1, HIGH); digitalWrite(DIRB2, LOW);  analogWrite(PWMB, pwm); } while (0)
#define MOTORB_STOP()       do { digitalWrite(DIRB1, LOW);  digitalWrite(DIRB2, LOW);  analogWrite(PWMB, 0); } while (0)

#define MOTORC_FORWARD(pwm) do { digitalWrite(DIRC1, LOW);  digitalWrite(DIRC2, HIGH); analogWrite(PWMC, pwm); } while (0)
#define MOTORC_BACKOFF(pwm) do { digitalWrite(DIRC1, HIGH); digitalWrite(DIRC2, LOW);  analogWrite(PWMC, pwm); } while (0)
#define MOTORC_STOP()       do { digitalWrite(DIRC1, LOW);  digitalWrite(DIRC2, LOW);  analogWrite(PWMC, 0); } while (0)

#define MOTORD_FORWARD(pwm) do { digitalWrite(DIRD1, LOW);  digitalWrite(DIRD2, HIGH); analogWrite(PWMD, pwm); } while (0)
#define MOTORD_BACKOFF(pwm) do { digitalWrite(DIRD1, HIGH); digitalWrite(DIRD2, LOW);  analogWrite(PWMD, pwm); } while (0)
#define MOTORD_STOP()       do { digitalWrite(DIRD1, LOW);  digitalWrite(DIRD2, LOW);  analogWrite(PWMD, 0); } while (0)

int motorPwm = 180;

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

// Same wheel directions as the original test file.
void moveForward() {
  MOTORA_FORWARD(motorPwm);
  MOTORB_BACKOFF(motorPwm);
  MOTORC_FORWARD(motorPwm);
  MOTORD_BACKOFF(motorPwm);
}

void moveBackward() {
  MOTORA_BACKOFF(motorPwm);
  MOTORB_FORWARD(motorPwm);
  MOTORC_BACKOFF(motorPwm);
  MOTORD_FORWARD(motorPwm);
}

void strafeLeft() {
  MOTORA_BACKOFF(motorPwm);
  MOTORB_BACKOFF(motorPwm);
  MOTORC_FORWARD(motorPwm);
  MOTORD_FORWARD(motorPwm);
}

void strafeRight() {
  MOTORA_FORWARD(motorPwm);
  MOTORB_FORWARD(motorPwm);
  MOTORC_BACKOFF(motorPwm);
  MOTORD_BACKOFF(motorPwm);
}

void rotateLeft() {
  MOTORA_BACKOFF(motorPwm);
  MOTORB_BACKOFF(motorPwm);
  MOTORC_BACKOFF(motorPwm);
  MOTORD_BACKOFF(motorPwm);
}

void rotateRight() {
  MOTORA_FORWARD(motorPwm);
  MOTORB_FORWARD(motorPwm);
  MOTORC_FORWARD(motorPwm);
  MOTORD_FORWARD(motorPwm);
}

void setupMotorPins() {
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);

  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);

  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
}

void printHelp() {
  Serial.println("Basic motor control ready.");
  Serial.println("Commands:");
  Serial.println("  F - forward");
  Serial.println("  B - backward");
  Serial.println("  L - strafe left");
  Serial.println("  R - strafe right");
  Serial.println("  Q - rotate left");
  Serial.println("  E - rotate right");
  Serial.println("  S - stop");
  Serial.println("  0..9 - set speed from 0 to 225");
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  stopCar();
  printHelp();
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F':
    case 'f':
      moveForward();
      Serial.println("Forward");
      break;
    case 'B':
    case 'b':
      moveBackward();
      Serial.println("Backward");
      break;
    case 'L':
    case 'l':
      strafeLeft();
      Serial.println("Left");
      break;
    case 'R':
    case 'r':
      strafeRight();
      Serial.println("Right");
      break;
    case 'Q':
    case 'q':
      rotateLeft();
      Serial.println("Rotate left");
      break;
    case 'E':
    case 'e':
      rotateRight();
      Serial.println("Rotate right");
      break;
    case 'S':
    case 's':
      stopCar();
      Serial.println("Stop");
      break;
    default:
      if (cmd >= '0' && cmd <= '9') {
        motorPwm = (cmd - '0') * 25;
        Serial.print("Speed set to ");
        Serial.println(motorPwm);
      }
      break;
  }
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
}
