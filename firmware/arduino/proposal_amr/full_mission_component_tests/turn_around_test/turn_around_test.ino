#include <Arduino.h>

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

const int TURN_PWM = 96;
const unsigned long TURN_MS = 700UL;

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

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

void rotateLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
}

void rotateRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

void timedTurn(bool leftTurn) {
  if (leftTurn) {
    Serial.println(F("Turning left."));
    rotateLeft(TURN_PWM);
  } else {
    Serial.println(F("Turning right."));
    rotateRight(TURN_PWM);
  }

  delay(TURN_MS);
  stopCar();
  Serial.println(F("Turn finished."));
}

void printHelp() {
  Serial.println(F("Turn around test ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  L - timed left turn"));
  Serial.println(F("  R - timed right turn"));
  Serial.println(F("  S - stop"));
  Serial.println(F("This is a bench test only. Replace timed turning with gyro-based heading logic later."));
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  stopCar();
  printHelp();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  char ch = static_cast<char>(Serial.read());
  switch (ch) {
    case 'L':
    case 'l':
      timedTurn(true);
      break;

    case 'R':
    case 'r':
      timedTurn(false);
      break;

    case 'S':
    case 's':
      stopCar();
      Serial.println(F("Stop."));
      break;

    default:
      break;
  }
}
