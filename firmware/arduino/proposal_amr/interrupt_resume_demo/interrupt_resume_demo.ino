#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

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

const uint8_t FRONT_ULTRASONIC_RIGHT_TRIG_PIN = 6;
const uint8_t FRONT_ULTRASONIC_RIGHT_ECHO_PIN = 4;
const uint8_t FRONT_ULTRASONIC_LEFT_TRIG_PIN = 10;
const uint8_t FRONT_ULTRASONIC_LEFT_ECHO_PIN = 7;
const uint8_t ARM_GRIP_SERVO_PIN = 44;
const uint8_t ARM_LIFT_SERVO_PIN = 45;
const uint8_t MPU6050_ADDR = 0x68;

const char *MODE_PROTOCOL_VERSION = "interrupt-resume-v1";
const char *TARGET_STATION = "STATION_A";

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long COMMAND_TIMEOUT_MS = 700UL;
const unsigned long ARM_ACTIVE_WINDOW_MS = 1200UL;
const unsigned long REMOTE_UNLOCK_OPEN_MS = 60000UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long START_SETTLE_MS = 250UL;
const unsigned long DOCK_STABLE_MS = 450UL;
const unsigned long VERIFY_STATION_MS = 1000UL;
const unsigned long REPORT_STATUS_MS = 800UL;
const unsigned long ARM_STEP_WAIT_MS = 550UL;

const int DEFAULT_MOTOR_PWM = 96;
const int MIN_MOTOR_PWM = 0;
const int MAX_MOTOR_PWM = 255;
const int DEFAULT_ARM_GRIP_DEG = 90;
const int DEFAULT_ARM_LIFT_DEG = 110;
const int ARM_GRIP_OPEN_DEG = 120;
const int ARM_GRIP_CLOSE_DEG = 60;
const int ARM_LIFT_PICK_DEG = 72;
const int MIN_ARM_DEG = 0;
const int MAX_ARM_DEG = 180;

const int AUTO_CORRIDOR_PWM = 92;
const int AUTO_APPROACH_PWM = 74;
const int AUTO_DOCK_PWM = 60;
const int AUTO_STRAFE_PWM = 66;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float HEADING_HOLD_KP = 2.2f;
const float HEADING_HOLD_DEADBAND_DEG = 0.8f;
const int HEADING_HOLD_MAX_PWM = 24;

const float CORRIDOR_TO_APPROACH_CM = 90.0f;
const float APPROACH_TO_DOCK_CM = 48.0f;
const float DOCK_TARGET_CM = 24.0f;
const float DOCK_DISTANCE_TOLERANCE_CM = 2.0f;
const float DOCK_ALIGN_TOLERANCE_CM = 2.5f;

const uint8_t MAX_INVALID_DISTANCE_COUNT = 10;
const uint8_t MAX_GYRO_READ_FAIL_COUNT = 8;

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

enum RunMode {
  MODE_AUTO = 0,
  MODE_MANUAL_OVERRIDE,
  MODE_FAULT
};

enum AutoStage {
  AUTO_STAGE_NONE = 0,
  AUTO_STAGE_IDLE,
  AUTO_STAGE_START,
  AUTO_STAGE_CORRIDOR_DRIVE,
  AUTO_STAGE_APPROACH_STATION,
  AUTO_STAGE_DOCK_AND_ALIGN,
  AUTO_STAGE_VERIFY_STATION,
  AUTO_STAGE_ARM_ACTION,
  AUTO_STAGE_REPORT_STATUS,
  AUTO_STAGE_DONE,
  AUTO_STAGE_FAULT
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
unsigned long autoStageStartedMs = 0;
unsigned long dockStableStartedMs = 0;
unsigned long armStepStartedMs = 0;

DriveDirection currentDirection = DRIVE_STOP;
RunMode currentMode = MODE_AUTO;
AutoStage currentAutoStage = AUTO_STAGE_IDLE;
AutoStage savedAutoStage = AUTO_STAGE_NONE;

bool commandTimedOut = false;
bool mpuReady = false;
bool taskActive = false;
bool resumePending = false;
bool manualSessionActive = false;
bool latestRfidMatch = false;
bool faultLatched = false;

uint8_t armActionStep = 0;
uint8_t invalidDistanceCount = 0;
uint8_t gyroReadFailCount = 0;

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

char commandBuffer[80];
uint8_t commandLength = 0;
char lastFault[64] = "";
char statusNote[64] = "";
char resumeNote[64] = "No saved task";
char latestRfidTag[24] = "";

void copyText(char *destination, size_t destinationSize, const char *source) {
  if (destinationSize == 0) {
    return;
  }
  strncpy(destination, source, destinationSize - 1);
  destination[destinationSize - 1] = '\0';
}

void setFaultText(const char *message) {
  copyText(lastFault, sizeof(lastFault), message);
}

void clearFaultText() {
  lastFault[0] = '\0';
}

void setStatusNote(const char *message) {
  copyText(statusNote, sizeof(statusNote), message);
}

void clearStatusNote() {
  statusNote[0] = '\0';
}

void setResumeNote(const char *message) {
  copyText(resumeNote, sizeof(resumeNote), message);
}

void clearRfidState() {
  latestRfidTag[0] = '\0';
  latestRfidMatch = false;
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
  gyroReadFailCount = 0;
}

void enterFault(const char *message);

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
    if (gyroReadFailCount < 255) {
      gyroReadFailCount++;
    }
    setFaultText("MPU6050 read fail");
    if (currentMode == MODE_AUTO && gyroReadFailCount >= MAX_GYRO_READ_FAIL_COUNT) {
      enterFault("MPU6050 read fail");
    }
    return;
  }

  gyroReadFailCount = 0;
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

void captureHeadingTarget() {
  headingTargetDeg = headingDeg;
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

void setMotionCommand(DriveDirection direction, int requestedPwm, bool refreshHeadingTarget) {
  DriveDirection previousDirection = currentDirection;
  motorPwm = constrain(requestedPwm, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  if (direction != DRIVE_STOP && motorPwm == 0) {
    direction = DRIVE_STOP;
  }

  currentDirection = direction;

  if (currentDirection != DRIVE_STOP &&
      (refreshHeadingTarget || previousDirection == DRIVE_STOP || previousDirection != currentDirection)) {
    captureHeadingTarget();
  }

  if (currentDirection == DRIVE_STOP) {
    lastHeadingCorrectionPwm = 0;
  }
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

const char *runModeName(RunMode mode) {
  switch (mode) {
    case MODE_AUTO:
      return "AUTO";
    case MODE_MANUAL_OVERRIDE:
      return "MANUAL_OVERRIDE";
    case MODE_FAULT:
    default:
      return "FAULT";
  }
}

const char *autoStageName(AutoStage stage) {
  switch (stage) {
    case AUTO_STAGE_IDLE:
      return "IDLE";
    case AUTO_STAGE_START:
      return "START";
    case AUTO_STAGE_CORRIDOR_DRIVE:
      return "CORRIDOR_DRIVE";
    case AUTO_STAGE_APPROACH_STATION:
      return "APPROACH_STATION";
    case AUTO_STAGE_DOCK_AND_ALIGN:
      return "DOCK_AND_ALIGN";
    case AUTO_STAGE_VERIFY_STATION:
      return "VERIFY_STATION";
    case AUTO_STAGE_ARM_ACTION:
      return "ARM_ACTION";
    case AUTO_STAGE_REPORT_STATUS:
      return "REPORT_STATUS";
    case AUTO_STAGE_DONE:
      return "DONE";
    case AUTO_STAGE_FAULT:
      return "FAULT";
    case AUTO_STAGE_NONE:
    default:
      return "";
  }
}

const char *savedStageName() {
  return autoStageName(savedAutoStage);
}

const char *stateName() {
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    if (currentDirection == DRIVE_STOP) {
      return "MANUAL_OVERRIDE";
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
        return "MANUAL_OVERRIDE";
    }
  }

  if (currentMode == MODE_FAULT) {
    return "FAULT";
  }

  if (remoteUnlockUntilMs > millis() && !taskActive) {
    return "REMOTE_UNLOCK";
  }

  const char *autoName = autoStageName(currentAutoStage);
  return autoName[0] == '\0' ? "IDLE" : autoName;
}

const char *deliveryResult() {
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    return resumePending ? "RESUME_PENDING" : "MANUAL_ONLY";
  }

  if (currentMode == MODE_FAULT || faultLatched) {
    return "FAULT";
  }

  if (currentAutoStage == AUTO_STAGE_DONE) {
    return "TASK_DONE";
  }

  if (currentAutoStage == AUTO_STAGE_VERIFY_STATION) {
    return latestRfidMatch ? "VERIFY_OK" : "VERIFYING";
  }

  if (currentAutoStage == AUTO_STAGE_ARM_ACTION) {
    return "ARM_SEQUENCE";
  }

  if (taskActive) {
    return "TASK_RUNNING";
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
  if (currentMode == MODE_FAULT) {
    return "FAULT";
  }
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    if (currentDirection != DRIVE_STOP) {
      return "MANUAL_ACTIVE";
    }
    if (commandTimedOut) {
      return "TIMEOUT";
    }
    return "MANUAL_READY";
  }
  if (taskActive) {
    if (currentDirection != DRIVE_STOP) {
      return "AUTO_ACTIVE";
    }
    return "AUTO_HOLD";
  }
  return "IDLE";
}

const char *armStatus() {
  if (lastArmCommandMs > 0 && millis() - lastArmCommandMs < ARM_ACTIVE_WINDOW_MS) {
    return "ACTIVE";
  }
  return "IDLE";
}

void clearTaskBookkeeping() {
  taskActive = false;
  resumePending = false;
  savedAutoStage = AUTO_STAGE_NONE;
  setResumeNote("No saved task");
}

bool frontDistanceValid() {
  return latestDistances.rightCm >= 0.0f && latestDistances.leftCm >= 0.0f && latestAverageCm >= 0.0f;
}

bool requireFrontDistance(const char *faultMessage) {
  if (frontDistanceValid()) {
    invalidDistanceCount = 0;
    return true;
  }

  if (invalidDistanceCount < 255) {
    invalidDistanceCount++;
  }

  setMotionCommand(DRIVE_STOP, motorPwm, false);
  if (invalidDistanceCount >= MAX_INVALID_DISTANCE_COUNT) {
    enterFault(faultMessage);
  } else {
    setStatusNote("Waiting for valid front distance");
  }
  return false;
}

AutoStage safeResumeStageFor(AutoStage stage) {
  switch (stage) {
    case AUTO_STAGE_START:
      return AUTO_STAGE_START;
    case AUTO_STAGE_CORRIDOR_DRIVE:
      return AUTO_STAGE_CORRIDOR_DRIVE;
    case AUTO_STAGE_APPROACH_STATION:
      return AUTO_STAGE_APPROACH_STATION;
    case AUTO_STAGE_DOCK_AND_ALIGN:
      return AUTO_STAGE_DOCK_AND_ALIGN;
    case AUTO_STAGE_VERIFY_STATION:
      return AUTO_STAGE_VERIFY_STATION;
    case AUTO_STAGE_ARM_ACTION:
      return AUTO_STAGE_ARM_ACTION;
    case AUTO_STAGE_REPORT_STATUS:
      return AUTO_STAGE_REPORT_STATUS;
    default:
      return AUTO_STAGE_NONE;
  }
}

void applyArmCommandInternal(const char *joint, int requestedDeg) {
  int angle = constrain(requestedDeg, MIN_ARM_DEG, MAX_ARM_DEG);
  if (strcmp(joint, "GRIP") == 0) {
    currentArmGripDeg = angle;
    armGripServo.write(currentArmGripDeg);
  } else {
    currentArmLiftDeg = angle;
    armLiftServo.write(currentArmLiftDeg);
  }
  lastArmCommandMs = millis();
}

void enterAutoStage(AutoStage stage) {
  currentAutoStage = stage;
  autoStageStartedMs = millis();
  dockStableStartedMs = 0;
  armActionStep = 0;
  invalidDistanceCount = 0;

  switch (stage) {
    case AUTO_STAGE_IDLE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      clearTaskBookkeeping();
      clearStatusNote();
      break;

    case AUTO_STAGE_START:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_CORRIDOR_PWM, false);
      captureHeadingTarget();
      clearRfidState();
      setStatusNote("Auto task start stage");
      break;

    case AUTO_STAGE_CORRIDOR_DRIVE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_FORWARD, AUTO_CORRIDOR_PWM, true);
      setStatusNote("Driving corridor stage");
      break;

    case AUTO_STAGE_APPROACH_STATION:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_FORWARD, AUTO_APPROACH_PWM, true);
      setStatusNote("Approaching station stage");
      break;

    case AUTO_STAGE_DOCK_AND_ALIGN:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Dock and align stage");
      break;

    case AUTO_STAGE_VERIFY_STATION:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      clearRfidState();
      setStatusNote("Verifying station stage");
      break;

    case AUTO_STAGE_ARM_ACTION:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Arm action restarted from safe entry");
      break;

    case AUTO_STAGE_REPORT_STATUS:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Reporting task status");
      break;

    case AUTO_STAGE_DONE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      clearTaskBookkeeping();
      setStatusNote("Auto task complete");
      break;

    case AUTO_STAGE_FAULT:
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;

    case AUTO_STAGE_NONE:
    default:
      break;
  }
}

void enterFault(const char *message) {
  stopCar();
  setMotionCommand(DRIVE_STOP, motorPwm, false);
  currentMode = MODE_FAULT;
  currentAutoStage = AUTO_STAGE_FAULT;
  taskActive = false;
  resumePending = false;
  savedAutoStage = AUTO_STAGE_NONE;
  manualSessionActive = false;
  faultLatched = true;
  setFaultText(message);
  setResumeNote("Resume blocked by fault");
  setStatusNote("Auto task faulted");
}

bool startAutoTask() {
  if (!mpuReady) {
    enterFault("MPU6050 not ready");
    return false;
  }

  if (currentMode == MODE_MANUAL_OVERRIDE &&
      (resumePending || taskActive || savedAutoStage != AUTO_STAGE_NONE)) {
    setStatusNote("Start rejected while manual is holding a saved task");
    return false;
  }

  clearFaultText();
  faultLatched = false;
  commandTimedOut = false;
  clearTaskBookkeeping();
  clearRfidState();
  applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
  applyArmCommandInternal("GRIP", DEFAULT_ARM_GRIP_DEG);
  enterAutoStage(AUTO_STAGE_START);
  return true;
}

void abortAutoTaskToIdle() {
  clearFaultText();
  faultLatched = false;
  clearRfidState();
  enterAutoStage(AUTO_STAGE_IDLE);
  setStatusNote("Auto task aborted to idle");
}

void enterManualOverride() {
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    setStatusNote("Manual override already active");
    return;
  }

  AutoStage safeStage = AUTO_STAGE_NONE;
  if (taskActive) {
    safeStage = safeResumeStageFor(currentAutoStage);
  }

  if (safeStage != AUTO_STAGE_NONE) {
    savedAutoStage = safeStage;
    resumePending = true;
    setResumeNote(autoStageName(safeStage));
  } else {
    savedAutoStage = AUTO_STAGE_NONE;
    resumePending = false;
    setResumeNote("No saved task");
  }

  stopCar();
  setMotionCommand(DRIVE_STOP, motorPwm, false);
  commandTimedOut = false;
  manualSessionActive = true;
  currentMode = MODE_MANUAL_OVERRIDE;

  if (faultLatched) {
    setStatusNote("Manual override active, fault still latched");
  } else if (resumePending) {
    setStatusNote("Manual override active, resume checkpoint saved");
  } else {
    setStatusNote("Manual override active, no resumable task");
  }
}

bool resumeAutoTask() {
  if (currentMode != MODE_MANUAL_OVERRIDE) {
    setStatusNote("Resume rejected, not in manual override");
    return false;
  }

  if (faultLatched) {
    setStatusNote("Resume rejected, fault is latched");
    return false;
  }

  if (!resumePending || savedAutoStage == AUTO_STAGE_NONE) {
    setStatusNote("Resume rejected, no saved stage");
    return false;
  }

  AutoStage stageToResume = savedAutoStage;
  savedAutoStage = AUTO_STAGE_NONE;
  resumePending = false;
  manualSessionActive = false;
  setResumeNote("No saved task");
  enterAutoStage(stageToResume);
  setStatusNote("Resumed from saved stage entry");
  return true;
}

void abortResumeWhileManual() {
  stopCar();
  setMotionCommand(DRIVE_STOP, motorPwm, false);
  clearTaskBookkeeping();
  currentAutoStage = AUTO_STAGE_IDLE;
  autoStageStartedMs = millis();
  manualSessionActive = true;
  currentMode = MODE_MANUAL_OVERRIDE;
  clearRfidState();
  setStatusNote("Saved task cleared, manual remains active");
}

void applyManualDriveCommand(DriveDirection direction, int requestedPwm) {
  if (currentMode != MODE_MANUAL_OVERRIDE) {
    setStatusNote("Drive rejected, take manual control first");
    return;
  }

  commandTimedOut = false;
  lastDriveCommandMs = millis();
  setMotionCommand(direction, requestedPwm, false);
}

void applyManualArmCommand(const char *joint, int requestedDeg) {
  if (currentMode != MODE_MANUAL_OVERRIDE) {
    setStatusNote("Arm rejected, take manual control first");
    return;
  }

  applyArmCommandInternal(joint, requestedDeg);
}

void updateDriveFailsafe() {
  if (currentMode != MODE_MANUAL_OVERRIDE || currentDirection == DRIVE_STOP) {
    return;
  }

  if (millis() - lastDriveCommandMs <= COMMAND_TIMEOUT_MS) {
    return;
  }

  setMotionCommand(DRIVE_STOP, motorPwm, false);
  commandTimedOut = true;
  stopCar();
  setStatusNote("Manual drive timeout");
}

void runArmActionStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);

  switch (armActionStep) {
    case 0:
      applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
      armStepStartedMs = millis();
      armActionStep = 1;
      break;

    case 1:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("GRIP", ARM_GRIP_OPEN_DEG);
        armStepStartedMs = millis();
        armActionStep = 2;
      }
      break;

    case 2:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("LIFT", ARM_LIFT_PICK_DEG);
        armStepStartedMs = millis();
        armActionStep = 3;
      }
      break;

    case 3:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("GRIP", ARM_GRIP_CLOSE_DEG);
        armStepStartedMs = millis();
        armActionStep = 4;
      }
      break;

    case 4:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
        armStepStartedMs = millis();
        armActionStep = 5;
      }
      break;

    case 5:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        enterAutoStage(AUTO_STAGE_REPORT_STATUS);
      }
      break;

    default:
      enterAutoStage(AUTO_STAGE_REPORT_STATUS);
      break;
  }
}

void runDockAndAlignStage() {
  if (!requireFrontDistance("Front distance invalid during dock")) {
    return;
  }

  if (fabs(latestDiffCm) > DOCK_ALIGN_TOLERANCE_CM) {
    if (latestDiffCm > 0.0f) {
      setMotionCommand(DRIVE_RIGHT, AUTO_STRAFE_PWM, false);
      setStatusNote("Dock align, strafing right");
    } else {
      setMotionCommand(DRIVE_LEFT, AUTO_STRAFE_PWM, false);
      setStatusNote("Dock align, strafing left");
    }
    dockStableStartedMs = 0;
    return;
  }

  if (latestAverageCm > DOCK_TARGET_CM + DOCK_DISTANCE_TOLERANCE_CM) {
    setMotionCommand(DRIVE_FORWARD, AUTO_DOCK_PWM, false);
    setStatusNote("Dock align, moving forward");
    dockStableStartedMs = 0;
    return;
  }

  if (latestAverageCm < DOCK_TARGET_CM - DOCK_DISTANCE_TOLERANCE_CM) {
    setMotionCommand(DRIVE_BACKWARD, AUTO_DOCK_PWM, false);
    setStatusNote("Dock align, backing off");
    dockStableStartedMs = 0;
    return;
  }

  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (dockStableStartedMs == 0) {
    dockStableStartedMs = millis();
    setStatusNote("Dock align, holding stable");
    return;
  }

  if (millis() - dockStableStartedMs >= DOCK_STABLE_MS) {
    enterAutoStage(AUTO_STAGE_VERIFY_STATION);
  }
}

void runAutoStateMachine() {
  if (currentMode != MODE_AUTO) {
    return;
  }

  switch (currentAutoStage) {
    case AUTO_STAGE_IDLE:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;

    case AUTO_STAGE_START:
      setMotionCommand(DRIVE_STOP, AUTO_CORRIDOR_PWM, false);
      if (millis() - autoStageStartedMs >= START_SETTLE_MS) {
        enterAutoStage(AUTO_STAGE_CORRIDOR_DRIVE);
      }
      break;

    case AUTO_STAGE_CORRIDOR_DRIVE:
      if (!requireFrontDistance("Front distance invalid in corridor")) {
        break;
      }
      setMotionCommand(DRIVE_FORWARD, AUTO_CORRIDOR_PWM, false);
      if (latestAverageCm <= CORRIDOR_TO_APPROACH_CM) {
        enterAutoStage(AUTO_STAGE_APPROACH_STATION);
      }
      break;

    case AUTO_STAGE_APPROACH_STATION:
      if (!requireFrontDistance("Front distance invalid while approaching")) {
        break;
      }
      setMotionCommand(DRIVE_FORWARD, AUTO_APPROACH_PWM, false);
      if (latestAverageCm <= APPROACH_TO_DOCK_CM) {
        enterAutoStage(AUTO_STAGE_DOCK_AND_ALIGN);
      }
      break;

    case AUTO_STAGE_DOCK_AND_ALIGN:
      runDockAndAlignStage();
      break;

    case AUTO_STAGE_VERIFY_STATION:
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      if (!latestRfidMatch && millis() - autoStageStartedMs >= VERIFY_STATION_MS / 2) {
        copyText(latestRfidTag, sizeof(latestRfidTag), TARGET_STATION);
        latestRfidMatch = true;
      }
      if (millis() - autoStageStartedMs >= VERIFY_STATION_MS) {
        enterAutoStage(AUTO_STAGE_ARM_ACTION);
      }
      break;

    case AUTO_STAGE_ARM_ACTION:
      runArmActionStage();
      break;

    case AUTO_STAGE_REPORT_STATUS:
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      if (millis() - autoStageStartedMs >= REPORT_STATUS_MS) {
        enterAutoStage(AUTO_STAGE_DONE);
      }
      break;

    case AUTO_STAGE_DONE:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;

    case AUTO_STAGE_FAULT:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;

    case AUTO_STAGE_NONE:
    default:
      break;
  }
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

void printJsonBool(bool value) {
  Serial.print(value ? "true" : "false");
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

  Serial.print("\"mode_protocol_version\":");
  printJsonString(MODE_PROTOCOL_VERSION);
  Serial.print(',');

  Serial.print("\"state\":");
  printJsonString(stateName());
  Serial.print(',');

  Serial.print("\"current_mode\":");
  printJsonString(runModeName(currentMode));
  Serial.print(',');

  Serial.print("\"current_auto_stage\":");
  printJsonString(autoStageName(currentAutoStage));
  Serial.print(',');

  Serial.print("\"saved_auto_stage\":");
  printJsonString(savedStageName());
  Serial.print(',');

  Serial.print("\"resume_pending\":");
  printJsonBool(resumePending);
  Serial.print(',');

  Serial.print("\"manual_session_active\":");
  printJsonBool(manualSessionActive);
  Serial.print(',');

  Serial.print("\"task_active\":");
  printJsonBool(taskActive);
  Serial.print(',');

  Serial.print("\"fault_latched\":");
  printJsonBool(faultLatched);
  Serial.print(',');

  Serial.print("\"resume_note\":");
  printJsonString(resumeNote);
  Serial.print(',');

  Serial.print("\"status_note\":");
  printJsonString(statusNote);
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
  printJsonString(latestRfidTag);
  Serial.print(',');

  Serial.print("\"rfid_match\":");
  printJsonBool(latestRfidMatch);
  Serial.print(',');

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
  char buffer[80];
  strncpy(buffer, command, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "OPEN_DOOR") == 0) {
    remoteUnlockUntilMs = millis() + REMOTE_UNLOCK_OPEN_MS;
    clearFaultText();
    return;
  }

  if (strcmp(token, "CLOSE_DOOR") == 0) {
    remoteUnlockUntilMs = 0;
    clearFaultText();
    return;
  }

  if (strcmp(token, "GYRO") == 0) {
    if (taskActive && currentMode == MODE_AUTO) {
      setStatusNote("GYRO rejected during active auto task");
      return;
    }
    if (!mpuReady) {
      setFaultText("MPU6050 not ready");
      return;
    }
    stopCar();
    calibrateGyroBias();
    clearFaultText();
    faultLatched = false;
    setStatusNote("Gyro recalibrated");
    return;
  }

  if (strcmp(token, "STOP") == 0) {
    applyManualDriveCommand(DRIVE_STOP, motorPwm);
    return;
  }

  if (strcmp(token, "TASK") == 0) {
    char *taskAction = strtok(NULL, " ");
    if (taskAction == NULL) {
      setStatusNote("Bad TASK command");
      return;
    }

    if (strcmp(taskAction, "START") == 0) {
      startAutoTask();
      return;
    }

    if (strcmp(taskAction, "ABORT") == 0) {
      if (currentMode == MODE_MANUAL_OVERRIDE) {
        abortResumeWhileManual();
      } else {
        abortAutoTaskToIdle();
      }
      return;
    }

    setStatusNote("Unknown TASK action");
    return;
  }

  if (strcmp(token, "OVERRIDE") == 0) {
    char *overrideAction = strtok(NULL, " ");
    if (overrideAction == NULL) {
      setStatusNote("Bad OVERRIDE command");
      return;
    }

    if (strcmp(overrideAction, "TAKE") == 0) {
      enterManualOverride();
      return;
    }

    if (strcmp(overrideAction, "RESUME") == 0) {
      resumeAutoTask();
      return;
    }

    if (strcmp(overrideAction, "ABORT") == 0) {
      if (currentMode != MODE_MANUAL_OVERRIDE) {
        setStatusNote("OVERRIDE ABORT rejected, not in manual");
        return;
      }
      abortResumeWhileManual();
      return;
    }

    setStatusNote("Unknown OVERRIDE action");
    return;
  }

  if (strcmp(token, "DRIVE") == 0) {
    char *directionToken = strtok(NULL, " ");
    char *powerToken = strtok(NULL, " ");
    if (directionToken == NULL) {
      setStatusNote("Bad DRIVE command");
      return;
    }

    bool ok = false;
    DriveDirection direction = parseDirection(directionToken, &ok);
    if (!ok) {
      setStatusNote("Bad DRIVE direction");
      return;
    }

    int requestedPwm = motorPwm;
    if (powerToken != NULL) {
      requestedPwm = atoi(powerToken);
    }

    applyManualDriveCommand(direction, requestedPwm);
    return;
  }

  if (strcmp(token, "ARM") == 0) {
    char *jointToken = strtok(NULL, " ");
    char *angleToken = strtok(NULL, " ");
    if (jointToken == NULL || angleToken == NULL) {
      setStatusNote("Bad ARM command");
      return;
    }

    int requestedDeg = atoi(angleToken);
    if (strcmp(jointToken, "LIFT") == 0) {
      applyManualArmCommand("LIFT", requestedDeg);
      return;
    }
    if (strcmp(jointToken, "GRIP") == 0) {
      applyManualArmCommand("GRIP", requestedDeg);
      return;
    }

    setStatusNote("Bad ARM joint");
    return;
  }

  setStatusNote("Unknown command");
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
  Serial.println(F("Interrupt/resume demo sketch ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  TASK START"));
  Serial.println(F("  TASK ABORT"));
  Serial.println(F("  OVERRIDE TAKE"));
  Serial.println(F("  OVERRIDE RESUME"));
  Serial.println(F("  OVERRIDE ABORT"));
  Serial.println(F("  DRIVE FORWARD <0-255>"));
  Serial.println(F("  DRIVE BACKWARD <0-255>"));
  Serial.println(F("  DRIVE LEFT <0-255>"));
  Serial.println(F("  DRIVE RIGHT <0-255>"));
  Serial.println(F("  DRIVE STOP <0-255>"));
  Serial.println(F("  ARM LIFT <0-180>"));
  Serial.println(F("  ARM GRIP <0-180>"));
  Serial.println(F("  GYRO"));
  Serial.println(F("  OPEN_DOOR"));
  Serial.println(F("  CLOSE_DOOR"));
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
    enterFault("MPU6050 init fail");
    Serial.println(F("MPU6050 init failed"));
  } else {
    mpuReady = true;
    calibrateGyroBias();
    enterAutoStage(AUTO_STAGE_IDLE);
  }

  delay(1000);
  printHelp();
}

void loop() {
  updateHeading();
  pollSerialCommands();
  updateDistanceSnapshot();
  runAutoStateMachine();
  updateDriveFailsafe();
  driveCurrentDirection();
  emitTelemetry();
}
