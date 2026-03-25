#include <Arduino.h>

// 这个程序用于测试小车直线前进时左右两边电机的 PWM 数值。
// 目标是找出一组 leftPwm 和 rightPwm，让小车前进时尽量不跑偏。
//
// 电机定义沿用 reference.ino：
// - 左侧轮组：A, C
// - 右侧轮组：B, D
// 前进时：
// - 左侧轮组使用 FORWARD
// - 右侧轮组使用 BACKOFF

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

int leftPwm = 90;
int rightPwm = 90;

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

void moveForwardWithBalance() {
  MOTORA_FORWARD(leftPwm);
  MOTORC_FORWARD(leftPwm);
  MOTORB_BACKOFF(rightPwm);
  MOTORD_BACKOFF(rightPwm);
}

void printStatus() {
  Serial.print("Left PWM = ");
  Serial.print(leftPwm);
  Serial.print(", Right PWM = ");
  Serial.println(rightPwm);
}

void printHelp() {
  Serial.println("Straight line power test ready.");
  Serial.println("Commands:");
  Serial.println("  F - forward with current left/right PWM");
  Serial.println("  S - stop");
  Serial.println("  A - left PWM + 1");
  Serial.println("  Z - left PWM - 1");
  Serial.println("  K - right PWM + 1");
  Serial.println("  M - right PWM - 1");
  Serial.println("  P - print current left/right PWM");
}

void clampPwmValues() {
  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F':
    case 'f':
      moveForwardWithBalance();
      Serial.println("Forward");
      printStatus();
      break;
    case 'S':
    case 's':
      stopCar();
      Serial.println("Stop");
      break;
    case 'A':
    case 'a':
      leftPwm += 1;
      clampPwmValues();
      printStatus();
      break;
    case 'Z':
    case 'z':
      leftPwm -= 1;
      clampPwmValues();
      printStatus();
      break;
    case 'K':
    case 'k':
      rightPwm += 1;
      clampPwmValues();
      printStatus();
      break;
    case 'M':
    case 'm':
      rightPwm -= 1;
      clampPwmValues();
      printStatus();
      break;
    case 'P':
    case 'p':
      printStatus();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  stopCar();
  printHelp();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }
}
