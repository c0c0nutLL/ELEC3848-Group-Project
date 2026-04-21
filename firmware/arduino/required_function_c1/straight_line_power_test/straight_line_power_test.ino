#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

// 这个测试草图用于在“无侧边 ultrasonic”条件下调直行参数。
// 核心思路：
// 1. 四个轮子分别给基础 PWM。
// 2. MPU6050 提供 heading error。
// 3. 运行时只做小幅 heading hold 修正，观察是否仍然跑偏。

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

const uint8_t MPU6050_ADDR = 0x68;
const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long STATUS_INTERVAL_MS = 250UL;
const float HEADING_HOLD_KP = 2.2f;
const float HEADING_HOLD_DEADBAND_DEG = 0.8f;
const int HEADING_HOLD_MAX_PWM = 24;

enum MotorIndex {
  MOTOR_A = 0,
  MOTOR_B,
  MOTOR_C,
  MOTOR_D,
  MOTOR_COUNT
};

int basePwm[MOTOR_COUNT] = {92, 84, 92, 84};
int currentWheelPwm[MOTOR_COUNT] = {0, 0, 0, 0};

bool mpuReady = false;
bool running = false;
unsigned long lastHeadingSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;
unsigned long lastStatusMs = 0;
float gyroZBias = 0.0f;
float latestGyroZDps = 0.0f;
float headingDeg = 0.0f;
float headingTargetDeg = 0.0f;
int lastHeadingCorrectionPwm = 0;

char serialBuffer[48];
uint8_t serialLength = 0;

void stopMotor(MotorIndex motor) {
  switch (motor) {
    case MOTOR_A:
      MOTORA_STOP();
      break;
    case MOTOR_B:
      MOTORB_STOP();
      break;
    case MOTOR_C:
      MOTORC_STOP();
      break;
    case MOTOR_D:
    default:
      MOTORD_STOP();
      break;
  }
  currentWheelPwm[motor] = 0;
}

void applySignedMotorPwm(MotorIndex motor, int signedPwm) {
  int pwm = constrain(abs(signedPwm), 0, 255);
  currentWheelPwm[motor] = pwm;

  if (signedPwm == 0 || pwm == 0) {
    stopMotor(motor);
    return;
  }

  switch (motor) {
    case MOTOR_A:
      if (signedPwm > 0) {
        MOTORA_FORWARD(pwm);
      } else {
        MOTORA_BACKOFF(pwm);
      }
      break;

    case MOTOR_B:
      if (signedPwm > 0) {
        MOTORB_FORWARD(pwm);
      } else {
        MOTORB_BACKOFF(pwm);
      }
      break;

    case MOTOR_C:
      if (signedPwm > 0) {
        MOTORC_FORWARD(pwm);
      } else {
        MOTORC_BACKOFF(pwm);
      }
      break;

    case MOTOR_D:
    default:
      if (signedPwm > 0) {
        MOTORD_FORWARD(pwm);
      } else {
        MOTORD_BACKOFF(pwm);
      }
      break;
  }
}

void stopCar() {
  stopMotor(MOTOR_A);
  stopMotor(MOTOR_B);
  stopMotor(MOTOR_C);
  stopMotor(MOTOR_D);
  lastHeadingCorrectionPwm = 0;
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

void writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readMpuBytes(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(MPU6050_ADDR, count);
  if (received != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMpu6050() {
  writeMpuRegister(0x6B, 0x00);
  writeMpuRegister(0x1B, 0x00);
  delay(100);

  uint8_t whoAmI = 0;
  if (!readMpuBytes(0x75, 1, &whoAmI)) {
    return false;
  }
  return whoAmI == 0x68;
}

float readGyroZDps() {
  uint8_t buffer[2];
  if (!readMpuBytes(0x47, 2, buffer)) {
    return 0.0f;
  }

  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  return static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB;
}

void calibrateGyroBias() {
  const int sampleCount = 400;
  float sum = 0.0f;

  stopCar();
  running = false;
  Serial.println(F("Calibrating gyro. Keep robot still."));
  delay(500);

  for (int i = 0; i < sampleCount; ++i) {
    sum += readGyroZDps();
    delay(4);
  }

  gyroZBias = sum / sampleCount;
  latestGyroZDps = 0.0f;
  headingDeg = 0.0f;
  headingTargetDeg = 0.0f;
  lastHeadingSampleMs = 0;
  lastHeadingUpdateUs = 0;

  Serial.print(F("Gyro bias(dps): "));
  Serial.println(gyroZBias, 4);
}

void updateHeading() {
  if (!mpuReady) {
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastHeadingSampleMs < HEADING_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastHeadingSampleMs = nowMs;

  unsigned long nowUs = micros();
  if (lastHeadingUpdateUs == 0) {
    lastHeadingUpdateUs = nowUs;
    return;
  }

  float dt = (nowUs - lastHeadingUpdateUs) / 1000000.0f;
  lastHeadingUpdateUs = nowUs;

  uint8_t buffer[2];
  if (!readMpuBytes(0x47, 2, buffer)) {
    latestGyroZDps = 0.0f;
    return;
  }

  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  float gyroZ = static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB - gyroZBias;
  latestGyroZDps = gyroZ;
  headingDeg += gyroZ * dt;
}

int computeHeadingCorrectionPwm() {
  if (!running || !mpuReady) {
    return 0;
  }

  float headingErrorDeg = headingTargetDeg - headingDeg;
  if (fabs(headingErrorDeg) <= HEADING_HOLD_DEADBAND_DEG) {
    return 0;
  }

  int averagePwm = (basePwm[MOTOR_A] + basePwm[MOTOR_B] + basePwm[MOTOR_C] + basePwm[MOTOR_D]) / MOTOR_COUNT;
  int maxCorrection = min(HEADING_HOLD_MAX_PWM, averagePwm / 3);
  int correction = static_cast<int>(headingErrorDeg * HEADING_HOLD_KP);
  return constrain(correction, -maxCorrection, maxCorrection);
}

void driveForwardWithHeadingHold() {
  if (!running) {
    stopCar();
    return;
  }

  lastHeadingCorrectionPwm = computeHeadingCorrectionPwm();
  applySignedMotorPwm(MOTOR_A, basePwm[MOTOR_A] - lastHeadingCorrectionPwm);
  applySignedMotorPwm(MOTOR_B, -basePwm[MOTOR_B] - lastHeadingCorrectionPwm);
  applySignedMotorPwm(MOTOR_C, basePwm[MOTOR_C] - lastHeadingCorrectionPwm);
  applySignedMotorPwm(MOTOR_D, -basePwm[MOTOR_D] - lastHeadingCorrectionPwm);
}

const char *motorName(MotorIndex motor) {
  switch (motor) {
    case MOTOR_A: return "A";
    case MOTOR_B: return "B";
    case MOTOR_C: return "C";
    case MOTOR_D: return "D";
    case MOTOR_COUNT:
    default: return "?";
  }
}

void printStatus() {
  Serial.print(F("RUN="));
  Serial.print(running ? F("ON") : F("OFF"));
  Serial.print(F(" A="));
  Serial.print(basePwm[MOTOR_A]);
  Serial.print(F(" B="));
  Serial.print(basePwm[MOTOR_B]);
  Serial.print(F(" C="));
  Serial.print(basePwm[MOTOR_C]);
  Serial.print(F(" D="));
  Serial.print(basePwm[MOTOR_D]);
  Serial.print(F(" heading="));
  Serial.print(headingDeg, 2);
  Serial.print(F(" target="));
  Serial.print(headingTargetDeg, 2);
  Serial.print(F(" err="));
  Serial.print(headingTargetDeg - headingDeg, 2);
  Serial.print(F(" corr="));
  Serial.print(lastHeadingCorrectionPwm);
  Serial.print(F(" gyroZ="));
  Serial.println(latestGyroZDps, 2);
}

void printHelp() {
  Serial.println(F("Straight line power test ready."));
  Serial.println(F("This sketch uses gyro heading hold + four-wheel PWM trim."));
  Serial.println(F("Commands:"));
  Serial.println(F("  RUN"));
  Serial.println(F("  STOP"));
  Serial.println(F("  ZERO      -> capture current heading as target"));
  Serial.println(F("  GYRO      -> recalibrate gyro bias"));
  Serial.println(F("  PRINT"));
  Serial.println(F("  SET A|B|C|D <PWM>"));
  Serial.println(F("  BUMP A|B|C|D <DELTA>"));
}

void uppercaseInPlace(char *text) {
  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    text[i] = static_cast<char>(toupper(static_cast<unsigned char>(text[i])));
  }
}

MotorIndex parseMotorToken(const char *token, bool *ok) {
  if (strcmp(token, "A") == 0) {
    *ok = true;
    return MOTOR_A;
  }
  if (strcmp(token, "B") == 0) {
    *ok = true;
    return MOTOR_B;
  }
  if (strcmp(token, "C") == 0) {
    *ok = true;
    return MOTOR_C;
  }
  if (strcmp(token, "D") == 0) {
    *ok = true;
    return MOTOR_D;
  }

  *ok = false;
  return MOTOR_A;
}

void handleCommandLine(char *line) {
  uppercaseInPlace(line);

  char *tokens[4] = {nullptr, nullptr, nullptr, nullptr};
  uint8_t tokenCount = 0;
  char *token = strtok(line, " ");
  while (token != nullptr && tokenCount < 4) {
    tokens[tokenCount++] = token;
    token = strtok(nullptr, " ");
  }

  if (tokenCount == 0) {
    return;
  }

  if (strcmp(tokens[0], "HELP") == 0 || strcmp(tokens[0], "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(tokens[0], "RUN") == 0 || strcmp(tokens[0], "F") == 0) {
    running = true;
    headingTargetDeg = headingDeg;
    Serial.println(F("Run"));
    printStatus();
    return;
  }

  if (strcmp(tokens[0], "STOP") == 0 || strcmp(tokens[0], "S") == 0) {
    running = false;
    stopCar();
    Serial.println(F("Stop"));
    printStatus();
    return;
  }

  if (strcmp(tokens[0], "ZERO") == 0) {
    headingTargetDeg = headingDeg;
    lastHeadingCorrectionPwm = 0;
    Serial.println(F("Heading target updated."));
    printStatus();
    return;
  }

  if (strcmp(tokens[0], "GYRO") == 0 || strcmp(tokens[0], "C") == 0) {
    if (!mpuReady) {
      Serial.println(F("MPU6050 not ready."));
      return;
    }
    calibrateGyroBias();
    printStatus();
    return;
  }

  if (strcmp(tokens[0], "PRINT") == 0 || strcmp(tokens[0], "P") == 0) {
    printStatus();
    return;
  }

  if ((strcmp(tokens[0], "SET") == 0 || strcmp(tokens[0], "BUMP") == 0) && tokenCount >= 3) {
    bool ok = false;
    MotorIndex motor = parseMotorToken(tokens[1], &ok);
    if (!ok) {
      Serial.println(F("Bad motor token."));
      return;
    }

    int value = atoi(tokens[2]);
    if (strcmp(tokens[0], "SET") == 0) {
      basePwm[motor] = constrain(value, 0, 255);
    } else {
      basePwm[motor] = constrain(basePwm[motor] + value, 0, 255);
    }

    printStatus();
    return;
  }

  Serial.println(F("Unknown command. Type HELP."));
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());

    if (ch == '\r' || ch == '\n') {
      if (serialLength > 0) {
        serialBuffer[serialLength] = '\0';
        handleCommandLine(serialBuffer);
        serialLength = 0;
      }
      continue;
    }

    if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = ch;
    }
  }
}

void printPeriodicStatus() {
  if (millis() - lastStatusMs < STATUS_INTERVAL_MS) {
    return;
  }
  lastStatusMs = millis();
  printStatus();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  setupMotorPins();
  stopCar();

  if (!initMpu6050()) {
    mpuReady = false;
    Serial.println(F("MPU6050 init failed"));
  } else {
    mpuReady = true;
    calibrateGyroBias();
  }

  printHelp();
  printStatus();
}

void loop() {
  updateHeading();
  processSerialInput();
  driveForwardWithHeadingHold();
  printPeriodicStatus();
}
