#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// 电机引脚沿用 Required Function 阶段已验证的映射。
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

// 这两组 ultrasonic 仍然是前保险杠右/左前方测距，不是已经移除的侧边超声波。
const uint8_t FRONT_ULTRASONIC_RIGHT_TRIG_PIN = 6;
const uint8_t FRONT_ULTRASONIC_RIGHT_ECHO_PIN = 4;
const uint8_t FRONT_ULTRASONIC_LEFT_TRIG_PIN = 10;
const uint8_t FRONT_ULTRASONIC_LEFT_ECHO_PIN = 7;
const uint8_t ARM_GRIP_SERVO_PIN = 44;
const uint8_t ARM_LIFT_SERVO_PIN = 45;
const uint8_t MPU6050_ADDR = 0x68;

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long COMMAND_TIMEOUT_MS = 700UL;
const unsigned long ARM_ACTIVE_WINDOW_MS = 1200UL;
const unsigned long REMOTE_UNLOCK_OPEN_MS = 5000UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;

const char *TARGET_STATION = "MANUAL";

const int DEFAULT_MOTOR_PWM = 96;
const int DEFAULT_ARM_GRIP_DEG = 90;
const int DEFAULT_ARM_LIFT_DEG = 110;
const int MIN_MOTOR_PWM = 0;
const int MAX_MOTOR_PWM = 255;
const int MIN_ARM_DEG = 0;
const int MAX_ARM_DEG = 180;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float HEADING_HOLD_KP = 2.2f;
const float HEADING_HOLD_DEADBAND_DEG = 0.8f;
const int HEADING_HOLD_MAX_PWM = 24;

enum DriveDirection {
  DRIVE_STOP = 0,
  DRIVE_FORWARD,
  DRIVE_BACKWARD,
  DRIVE_LEFT,
  DRIVE_RIGHT
};

enum MotorIndex {
  MOTOR_A = 0,
  MOTOR_B,
  MOTOR_C,
  MOTOR_D,
  MOTOR_COUNT
};

struct DistanceSnapshot {
  float rightCm;
  float leftCm;
};

struct DirectionTrim {
  int8_t offset[MOTOR_COUNT];
};

const int8_t DRIVE_SIGN[][MOTOR_COUNT] = {
  {0, 0, 0, 0},
  {1, -1, 1, -1},
  {-1, 1, -1, 1},
  {-1, -1, 1, 1},
  {1, 1, -1, -1}
};

// 四轮基础 trim 用来补偿底盘天然偏航，动态 heading hold 只做小幅修正。
const DirectionTrim DEFAULT_DIRECTION_TRIM[] = {
  {{0, 0, 0, 0}},
  {{8, 0, 8, 0}},
  {{8, 0, 8, 0}},
  {{0, 2, 0, 0}},
  {{0, 2, 0, 0}}
};

unsigned long lastTelemetryMs = 0;
unsigned long lastDistanceSampleMs = 0;
unsigned long lastDriveCommandMs = 0;
unsigned long lastArmCommandMs = 0;
unsigned long remoteUnlockUntilMs = 0;
unsigned long lastHeadingSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;

DriveDirection currentDirection = DRIVE_STOP;
bool commandTimedOut = false;
bool mpuReady = false;
int motorPwm = DEFAULT_MOTOR_PWM;
int currentArmGripDeg = DEFAULT_ARM_GRIP_DEG;
int currentArmLiftDeg = DEFAULT_ARM_LIFT_DEG;
int currentWheelPwm[MOTOR_COUNT] = {0, 0, 0, 0};

DistanceSnapshot latestDistances = {-1.0f, -1.0f};
float latestAverageCm = -1.0f;
float latestDiffCm = -1.0f;
float gyroZBias = 0.0f;
float latestGyroZDps = 0.0f;
float headingDeg = 0.0f;
float headingTargetDeg = 0.0f;
int lastHeadingCorrectionPwm = 0;

Servo armGripServo;
Servo armLiftServo;

char commandBuffer[48];
uint8_t commandLength = 0;
char lastFault[48] = "";

void setFault(const char *message) {
  strncpy(lastFault, message, sizeof(lastFault) - 1);
  lastFault[sizeof(lastFault) - 1] = '\0';
}

void clearFault() {
  lastFault[0] = '\0';
}

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
  int pwm = constrain(abs(signedPwm), MIN_MOTOR_PWM, MAX_MOTOR_PWM);
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

void setupFrontUltrasonics() {
  pinMode(FRONT_ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ULTRASONIC_RIGHT_ECHO_PIN, INPUT);
  pinMode(FRONT_ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ULTRASONIC_LEFT_ECHO_PIN, INPUT);

  digitalWrite(FRONT_ULTRASONIC_RIGHT_TRIG_PIN, LOW);
  digitalWrite(FRONT_ULTRASONIC_LEFT_TRIG_PIN, LOW);
}

void setupArm() {
  armGripServo.attach(ARM_GRIP_SERVO_PIN);
  armLiftServo.attach(ARM_LIFT_SERVO_PIN);
  armGripServo.write(currentArmGripDeg);
  armLiftServo.write(currentArmLiftDeg);
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
    setFault("MPU6050 read fail");
    latestGyroZDps = 0.0f;
    return;
  }

  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  float gyroZ = static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB - gyroZBias;
  latestGyroZDps = gyroZ;
  headingDeg += gyroZ * dt;
}

float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f * 0.5f;
}

void updateDistanceSnapshot() {
  if (millis() - lastDistanceSampleMs < DISTANCE_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastDistanceSampleMs = millis();
  latestDistances.rightCm = readDistanceCm(FRONT_ULTRASONIC_RIGHT_TRIG_PIN, FRONT_ULTRASONIC_RIGHT_ECHO_PIN);
  delay(4);
  latestDistances.leftCm = readDistanceCm(FRONT_ULTRASONIC_LEFT_TRIG_PIN, FRONT_ULTRASONIC_LEFT_ECHO_PIN);

  if (latestDistances.rightCm >= 0.0f && latestDistances.leftCm >= 0.0f) {
    latestAverageCm = (latestDistances.rightCm + latestDistances.leftCm) * 0.5f;
    latestDiffCm = latestDistances.rightCm - latestDistances.leftCm;
  } else {
    latestAverageCm = -1.0f;
    latestDiffCm = -1.0f;
  }
}

int computeHeadingCorrectionPwm() {
  if (!mpuReady || currentDirection == DRIVE_STOP) {
    return 0;
  }

  float headingErrorDeg = headingTargetDeg - headingDeg;
  if (fabs(headingErrorDeg) <= HEADING_HOLD_DEADBAND_DEG) {
    return 0;
  }

  int maxCorrection = min(HEADING_HOLD_MAX_PWM, motorPwm / 3);
  int correction = static_cast<int>(headingErrorDeg * HEADING_HOLD_KP);
  return constrain(correction, -maxCorrection, maxCorrection);
}

void captureHeadingTarget() {
  headingTargetDeg = headingDeg;
}

void driveCurrentDirection() {
  if (currentDirection == DRIVE_STOP || motorPwm <= 0) {
    stopCar();
    return;
  }

  int correctionPwm = computeHeadingCorrectionPwm();
  lastHeadingCorrectionPwm = correctionPwm;
  const DirectionTrim &trim = DEFAULT_DIRECTION_TRIM[currentDirection];

  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    int basePwm = constrain(motorPwm + trim.offset[motor], MIN_MOTOR_PWM, MAX_MOTOR_PWM);
    int signedPwm = DRIVE_SIGN[currentDirection][motor] * basePwm - correctionPwm;
    applySignedMotorPwm(static_cast<MotorIndex>(motor), signedPwm);
  }
}

DriveDirection parseDirection(const char *token, bool *ok) {
  if (strcmp(token, "FORWARD") == 0) {
    *ok = true;
    return DRIVE_FORWARD;
  }
  if (strcmp(token, "BACKWARD") == 0) {
    *ok = true;
    return DRIVE_BACKWARD;
  }
  if (strcmp(token, "LEFT") == 0) {
    *ok = true;
    return DRIVE_LEFT;
  }
  if (strcmp(token, "RIGHT") == 0) {
    *ok = true;
    return DRIVE_RIGHT;
  }
  if (strcmp(token, "STOP") == 0) {
    *ok = true;
    return DRIVE_STOP;
  }

  *ok = false;
  return DRIVE_STOP;
}

const char *directionName(DriveDirection direction) {
  switch (direction) {
    case DRIVE_FORWARD:
      return "FORWARD";
    case DRIVE_BACKWARD:
      return "BACKWARD";
    case DRIVE_LEFT:
      return "LEFT";
    case DRIVE_RIGHT:
      return "RIGHT";
    case DRIVE_STOP:
    default:
      return "STOP";
  }
}

const char *stateName() {
  if (currentDirection == DRIVE_STOP) {
    if (remoteUnlockUntilMs > millis()) {
      return "REMOTE_UNLOCK";
    }
    return "IDLE";
  }

  switch (currentDirection) {
    case DRIVE_FORWARD:
      return "MANUAL_FORWARD";
    case DRIVE_BACKWARD:
      return "MANUAL_BACKWARD";
    case DRIVE_LEFT:
      return "MANUAL_LEFT";
    case DRIVE_RIGHT:
      return "MANUAL_RIGHT";
    case DRIVE_STOP:
    default:
      return "IDLE";
  }
}

const char *deliveryResult() {
  if (currentDirection != DRIVE_STOP) {
    return "MANUAL_CONTROL";
  }
  if (remoteUnlockUntilMs > millis()) {
    return "UNLOCKED";
  }
  return "READY";
}

const char *cargoDoorState() {
  return remoteUnlockUntilMs > millis() ? "OPEN" : "CLOSED";
}

const char *controlStatus() {
  if (currentDirection != DRIVE_STOP) {
    return "ACTIVE";
  }
  if (commandTimedOut) {
    return "TIMEOUT";
  }
  return "IDLE";
}

const char *armStatus() {
  if (lastArmCommandMs > 0 && millis() - lastArmCommandMs < ARM_ACTIVE_WINDOW_MS) {
    return "ACTIVE";
  }
  return "IDLE";
}

void applyDriveCommand(DriveDirection direction, int requestedPwm) {
  DriveDirection previousDirection = currentDirection;

  motorPwm = constrain(requestedPwm, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  if (direction != DRIVE_STOP && motorPwm == 0) {
    direction = DRIVE_STOP;
  }

  currentDirection = direction;
  lastDriveCommandMs = millis();
  commandTimedOut = false;
  clearFault();

  if (currentDirection != DRIVE_STOP &&
      (previousDirection == DRIVE_STOP || previousDirection != currentDirection)) {
    captureHeadingTarget();
  }

  driveCurrentDirection();
}

void applyArmCommand(const char *joint, int requestedDeg) {
  int angle = constrain(requestedDeg, MIN_ARM_DEG, MAX_ARM_DEG);
  if (strcmp(joint, "GRIP") == 0) {
    currentArmGripDeg = angle;
    armGripServo.write(currentArmGripDeg);
  } else {
    currentArmLiftDeg = angle;
    armLiftServo.write(currentArmLiftDeg);
  }
  lastArmCommandMs = millis();
  clearFault();
}

void updateDriveFailsafe() {
  if (currentDirection == DRIVE_STOP) {
    return;
  }

  if (millis() - lastDriveCommandMs <= COMMAND_TIMEOUT_MS) {
    return;
  }

  currentDirection = DRIVE_STOP;
  commandTimedOut = true;
  stopCar();
}

void printJsonString(const char *value) {
  Serial.print('"');
  Serial.print(value);
  Serial.print('"');
}

void printJsonFloat(float value, uint8_t precision) {
  if (value < 0.0f) {
    Serial.print("null");
    return;
  }
  Serial.print(value, precision);
}

void emitTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = millis();

  Serial.print('{');

  Serial.print("\"ts_ms\":");
  Serial.print(millis());
  Serial.print(',');

  Serial.print("\"state\":");
  printJsonString(stateName());
  Serial.print(',');

  Serial.print("\"target_station\":");
  printJsonString(TARGET_STATION);
  Serial.print(',');

  Serial.print("\"right_cm\":");
  printJsonFloat(latestDistances.rightCm, 1);
  Serial.print(',');

  Serial.print("\"left_cm\":");
  printJsonFloat(latestDistances.leftCm, 1);
  Serial.print(',');

  Serial.print("\"avg_cm\":");
  printJsonFloat(latestAverageCm, 1);
  Serial.print(',');

  Serial.print("\"diff_cm\":");
  printJsonFloat(latestDiffCm, 1);
  Serial.print(',');

  Serial.print("\"heading_deg\":");
  Serial.print(headingDeg, 2);
  Serial.print(',');

  Serial.print("\"heading_target_deg\":");
  Serial.print(headingTargetDeg, 2);
  Serial.print(',');

  Serial.print("\"heading_error_deg\":");
  Serial.print(headingTargetDeg - headingDeg, 2);
  Serial.print(',');

  Serial.print("\"gyro_z_dps\":");
  Serial.print(latestGyroZDps, 2);
  Serial.print(',');

  Serial.print("\"heading_correction_pwm\":");
  Serial.print(lastHeadingCorrectionPwm);
  Serial.print(',');

  Serial.print("\"rfid_tag\":");
  printJsonString("");
  Serial.print(',');

  Serial.print("\"rfid_match\":false,");

  Serial.print("\"cargo_door\":");
  printJsonString(cargoDoorState());
  Serial.print(',');

  Serial.print("\"delivery_result\":");
  printJsonString(deliveryResult());
  Serial.print(',');

  Serial.print("\"control_direction\":");
  printJsonString(directionName(currentDirection));
  Serial.print(',');

  Serial.print("\"motor_pwm\":");
  Serial.print(motorPwm);
  Serial.print(',');

  Serial.print("\"motor_pwm_a\":");
  Serial.print(currentWheelPwm[MOTOR_A]);
  Serial.print(',');

  Serial.print("\"motor_pwm_b\":");
  Serial.print(currentWheelPwm[MOTOR_B]);
  Serial.print(',');

  Serial.print("\"motor_pwm_c\":");
  Serial.print(currentWheelPwm[MOTOR_C]);
  Serial.print(',');

  Serial.print("\"motor_pwm_d\":");
  Serial.print(currentWheelPwm[MOTOR_D]);
  Serial.print(',');

  Serial.print("\"control_status\":");
  printJsonString(controlStatus());
  Serial.print(',');

  Serial.print("\"arm_lift_deg\":");
  Serial.print(currentArmLiftDeg);
  Serial.print(',');

  Serial.print("\"arm_grip_deg\":");
  Serial.print(currentArmGripDeg);
  Serial.print(',');

  Serial.print("\"arm_status\":");
  printJsonString(armStatus());
  Serial.print(',');

  Serial.print("\"command_age_ms\":");
  if (lastDriveCommandMs == 0) {
    Serial.print("null");
  } else {
    Serial.print(millis() - lastDriveCommandMs);
  }
  Serial.print(',');

  Serial.print("\"fault\":");
  printJsonString(lastFault);

  Serial.println('}');
}

void handleCommand(const char *command) {
  char buffer[48];
  strncpy(buffer, command, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "OPEN_DOOR") == 0) {
    remoteUnlockUntilMs = millis() + REMOTE_UNLOCK_OPEN_MS;
    clearFault();
    return;
  }

  if (strcmp(token, "STOP") == 0) {
    applyDriveCommand(DRIVE_STOP, motorPwm);
    return;
  }

  if (strcmp(token, "GYRO") == 0) {
    if (!mpuReady) {
      setFault("MPU6050 not ready");
      return;
    }
    clearFault();
    calibrateGyroBias();
    return;
  }

  if (strcmp(token, "DRIVE") == 0) {
    char *directionToken = strtok(NULL, " ");
    char *powerToken = strtok(NULL, " ");
    if (directionToken == NULL) {
      setFault("BAD_DRIVE_COMMAND");
      return;
    }

    bool ok = false;
    DriveDirection direction = parseDirection(directionToken, &ok);
    if (!ok) {
      setFault("BAD_DIRECTION");
      return;
    }

    int requestedPwm = motorPwm;
    if (powerToken != NULL) {
      requestedPwm = atoi(powerToken);
    }

    applyDriveCommand(direction, requestedPwm);
    return;
  }

  if (strcmp(token, "ARM") == 0) {
    char *jointToken = strtok(NULL, " ");
    char *angleToken = strtok(NULL, " ");
    if (jointToken == NULL || angleToken == NULL) {
      setFault("BAD_ARM_COMMAND");
      return;
    }

    int requestedDeg = atoi(angleToken);
    if (strcmp(jointToken, "LIFT") == 0) {
      applyArmCommand("LIFT", requestedDeg);
      return;
    }
    if (strcmp(jointToken, "GRIP") == 0) {
      applyArmCommand("GRIP", requestedDeg);
      return;
    }

    setFault("BAD_ARM_JOINT");
    return;
  }

  setFault("UNKNOWN_COMMAND");
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0) {
        handleCommand(commandBuffer);
      }
      commandLength = 0;
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = ch;
    }
  }
}

void printHelp() {
  Serial.println(F("Manual telemetry sketch ready."));
  Serial.println(F("Front ultrasonic pair is still active for docking/telemetry."));
  Serial.println(F("Drive control now uses gyro heading hold + per-wheel trim."));
  Serial.println(F("Commands:"));
  Serial.println(F("  DRIVE FORWARD <0-255>"));
  Serial.println(F("  DRIVE BACKWARD <0-255>"));
  Serial.println(F("  DRIVE LEFT <0-255>"));
  Serial.println(F("  DRIVE RIGHT <0-255>"));
  Serial.println(F("  DRIVE STOP <0-255>"));
  Serial.println(F("  ARM LIFT <0-180>"));
  Serial.println(F("  ARM GRIP <0-180>"));
  Serial.println(F("  GYRO"));
  Serial.println(F("  OPEN_DOOR"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  setupMotorPins();
  setupFrontUltrasonics();
  setupArm();
  stopCar();

  if (!initMpu6050()) {
    mpuReady = false;
    setFault("MPU6050 init fail");
    Serial.println(F("MPU6050 init failed"));
  } else {
    mpuReady = true;
    calibrateGyroBias();
  }

  delay(1000);
  printHelp();
}

void loop() {
  updateHeading();
  pollSerialCommands();
  updateDriveFailsafe();
  updateDistanceSnapshot();
  driveCurrentDirection();
  emitTelemetry();
}
