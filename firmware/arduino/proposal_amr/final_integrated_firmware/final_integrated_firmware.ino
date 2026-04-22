#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

typedef unsigned char uchar;
typedef unsigned int uint;

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

// Pin mapping source of truth: hardware/amr_pin_mapping_template.xlsx
// Rows applied directly from the workbook:
// - front ultrasonic pair: D6 / D4 / D10 / D7
// - RFID SPI SS: D53, red LED: D41, green LED: D47, door servo: D13
// - arm lift servo: D44, arm grip servo: D45
// This intentionally follows the spreadsheet even though older docs in plan.md / agent.md list the arm pins the other way around.
const uint8_t FRONT_ULTRASONIC_RIGHT_TRIG_PIN = 6;
const uint8_t FRONT_ULTRASONIC_RIGHT_ECHO_PIN = 4;
const uint8_t FRONT_ULTRASONIC_LEFT_TRIG_PIN = 10;
const uint8_t FRONT_ULTRASONIC_LEFT_ECHO_PIN = 7;
const uint8_t RFID_SS_PIN = 53;
const uint8_t RED_LED_PIN = 41;
const uint8_t GREEN_LED_PIN = 47;
const uint8_t CARGO_DOOR_SERVO_PIN = 13;
const uint8_t ARM_LIFT_SERVO_PIN = 44;
const uint8_t ARM_GRIP_SERVO_PIN = 45;

const uint8_t RFID_RST_PIN = 5;

const uint8_t MPU6050_ADDR = 0x68;
const char *MODE_PROTOCOL_VERSION = "mission-runtime-v1";

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long COMMAND_TIMEOUT_MS = 700UL;
const unsigned long ARM_ACTIVE_WINDOW_MS = 1200UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long RFID_POLL_INTERVAL_MS = 120UL;
const unsigned long START_SETTLE_MS = 300UL;
const unsigned long VERIFY_SETTLE_MS = 300UL;
const unsigned long VERIFY_TIMEOUT_MS = 8000UL;
const unsigned long DOCK_STABLE_MS = 450UL;
const unsigned long ARM_STEP_WAIT_MS = 550UL;
const unsigned long DOOR_SETTLE_MS = 900UL;
const unsigned long LOAD_SETTLE_MS = 2500UL;
const unsigned long TURN_AROUND_MS = 720UL;
const unsigned long REPORT_STATUS_MS = 800UL;
const unsigned long DELIVERY_DOOR_AUTO_CLOSE_MS = 60000UL;
const unsigned long OBSTACLE_STOP_HOLD_MS = 350UL;
const unsigned long OBSTACLE_STRAFE_MS = 850UL;
const unsigned long OBSTACLE_PASS_MS = 1200UL;
const unsigned long OBSTACLE_RETURN_MS = 850UL;

const int DEFAULT_MOTOR_PWM = 96;
const int MIN_MOTOR_PWM = 0;
const int MAX_MOTOR_PWM = 255;
const int AUTO_TRAVEL_PWM = 92;
const int AUTO_APPROACH_PWM = 74;
const int AUTO_DOCK_PWM = 60;
const int AUTO_STRAFE_PWM = 66;
const int AUTO_TURN_PWM = 96;

const int DEFAULT_ARM_LIFT_DEG = 110;
const int DEFAULT_ARM_GRIP_DEG = 90;
const int ARM_LIFT_PICK_DEG = 72;
const int ARM_GRIP_OPEN_DEG = 120;
const int ARM_GRIP_CLOSE_DEG = 60;
const int CARGO_DOOR_OPEN_DEG = 108;
const int CARGO_DOOR_CLOSED_DEG = 18;
const int MIN_ARM_DEG = 0;
const int MAX_ARM_DEG = 180;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float HEADING_HOLD_KP = 2.2f;
const float HEADING_HOLD_DEADBAND_DEG = 0.8f;
const int HEADING_HOLD_MAX_PWM = 24;

const float CORRIDOR_TO_APPROACH_CM = 90.0f;
const float APPROACH_TO_DOCK_CM = 48.0f;
const float DOCK_TARGET_CM = 24.0f;
const float DOCK_DISTANCE_TOLERANCE_CM = 2.0f;
const float DOCK_ALIGN_TOLERANCE_CM = 2.5f;

// TODO(obstacle-threshold): final obstacle trigger and recovery distances still need hardware tuning.
const float OBSTACLE_TRIGGER_CM = 18.0f;
const float OBSTACLE_CLEAR_CM = 28.0f;

const uint8_t MAX_INVALID_DISTANCE_COUNT = 10;
const uint8_t MAX_GYRO_READ_FAIL_COUNT = 8;

const char PICKUP_RFID_UID[] = "7B97F506";
const char DELIVERY_RFID_UID[] = "6C263307";

enum DriveDirection {
  DRIVE_STOP = 0,
  DRIVE_FORWARD,
  DRIVE_BACKWARD,
  DRIVE_LEFT,
  DRIVE_RIGHT,
  DRIVE_TURN_LEFT,
  DRIVE_TURN_RIGHT
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

enum MissionTask {
  TASK_NONE = 0,
  TASK_FULL_MISSION,
  TASK_PICKUP_MISSION,
  TASK_DELIVERY_MISSION,
  TASK_MANUAL_TEST
};

enum AutoStage {
  STAGE_NONE = 0,
  STAGE_IDLE,
  STAGE_START,
  STAGE_TRAVEL_TO_PICKUP,
  STAGE_VERIFY_PICKUP_STATION,
  STAGE_PICKUP_SEQUENCE,
  STAGE_OPEN_DOOR_FOR_LOAD,
  STAGE_LOAD_CARGO,
  STAGE_CLOSE_DOOR_AFTER_LOAD,
  STAGE_TURN_AROUND,
  STAGE_TRAVEL_TO_DELIVERY,
  STAGE_VERIFY_DELIVERY_STATION,
  STAGE_WAIT_USER_UNLOCK,
  STAGE_OPEN_DOOR_FOR_UNLOAD,
  STAGE_WAIT_DOOR_CLOSE,
  STAGE_AUTO_CLOSE_DOOR,
  STAGE_REPORT_STATUS,
  STAGE_DONE,
  STAGE_FAULT
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
  {1, 1, -1, -1},
  {-1, -1, -1, -1},
  {1, 1, 1, 1}
};

const DirectionTrim DEFAULT_DIRECTION_TRIM[] = {
  {{0, 0, 0, 0}},
  {{8, 0, 8, 0}},
  {{8, 0, 8, 0}},
  {{0, 2, 0, 0}},
  {{0, 2, 0, 0}},
  {{0, 0, 0, 0}},
  {{0, 0, 0, 0}}
};

// Minimal MFRC522 register set for polling UID over SPI.
#define MAX_LEN 16

#define PCD_IDLE 0x00
#define PCD_TRANSCEIVE 0x0C
#define PCD_RESETPHASE 0x0F

#define PICC_REQIDL 0x26
#define PICC_ANTICOLL 0x93

#define MI_OK 0
#define MI_NOTAGERR 1
#define MI_ERR 2

#define CommandReg 0x01
#define CommIEnReg 0x02
#define CommIrqReg 0x04
#define ErrorReg 0x06
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define ModeReg 0x11
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D

unsigned long lastTelemetryMs = 0;
unsigned long lastDistanceSampleMs = 0;
unsigned long lastHeadingSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;
unsigned long lastDriveCommandMs = 0;
unsigned long lastArmCommandMs = 0;
unsigned long lastRfidPollMs = 0;
unsigned long autoStageStartedMs = 0;
unsigned long dockStableStartedMs = 0;
unsigned long armStepStartedMs = 0;
unsigned long unloadDoorOpenedMs = 0;
unsigned long obstacleStepStartedMs = 0;

DriveDirection currentDirection = DRIVE_STOP;
RunMode currentMode = MODE_AUTO;
MissionTask currentTask = TASK_NONE;
AutoStage currentAutoStage = STAGE_IDLE;
AutoStage savedAutoStage = STAGE_NONE;

bool mpuReady = false;
bool commandTimedOut = false;
bool taskActive = false;
bool resumePending = false;
bool manualSessionActive = false;
bool faultLatched = false;
bool cargoDoorOpen = false;
bool pickupRfidMatched = false;
bool deliveryRfidMatched = false;
bool unlockApproved = false;
bool doorManualCloseRequested = false;
bool doorAutoClosed = false;
bool obstacleRoutineActive = false;
bool lastRfidCardPresent = false;
bool redLedOn = false;
bool greenLedOn = false;

uint8_t invalidDistanceCount = 0;
uint8_t gyroReadFailCount = 0;
uint8_t pickupSequenceStep = 0;
uint8_t obstacleRoutineStep = 0;

int motorPwm = DEFAULT_MOTOR_PWM;
int currentArmLiftDeg = DEFAULT_ARM_LIFT_DEG;
int currentArmGripDeg = DEFAULT_ARM_GRIP_DEG;
int currentWheelPwm[MOTOR_COUNT] = {0, 0, 0, 0};
int lastHeadingCorrectionPwm = 0;

DistanceSnapshot latestDistances = {-1.0f, -1.0f};
float latestAverageCm = -1.0f;
float latestDiffCm = -1.0f;
float gyroZBias = 0.0f;
float latestGyroZDps = 0.0f;
float headingDeg = 0.0f;
float headingTargetDeg = 0.0f;

Servo armLiftServo;
Servo armGripServo;
Servo cargoDoorServo;

char commandBuffer[96];
uint8_t commandLength = 0;
char lastFault[64] = "";
char statusNote[96] = "";
char resumeNote[64] = "No saved task";
char latestRfidTag[24] = "";
char lastRfidStation[24] = "NONE";
char lastObservedUid[24] = "";

void copyText(char *destination, size_t destinationSize, const char *source) {
  if (destinationSize == 0) {
    return;
  }
  if (source == NULL) {
    source = "";
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

void clearResumeState() {
  savedAutoStage = STAGE_NONE;
  resumePending = false;
  setResumeNote("No saved task");
}

void clearRfidState() {
  latestRfidTag[0] = '\0';
  lastObservedUid[0] = '\0';
  copyText(lastRfidStation, sizeof(lastRfidStation), "NONE");
  pickupRfidMatched = false;
  deliveryRfidMatched = false;
  lastRfidCardPresent = false;
}

const char *taskName(MissionTask task) {
  switch (task) {
    case TASK_FULL_MISSION:
      return "FULL_MISSION";
    case TASK_PICKUP_MISSION:
      return "PICKUP_MISSION";
    case TASK_DELIVERY_MISSION:
      return "DELIVERY_MISSION";
    case TASK_MANUAL_TEST:
      return "MANUAL_TEST";
    case TASK_NONE:
    default:
      return "";
  }
}

MissionTask parseTaskId(const char *token) {
  if (token == NULL) {
    return TASK_FULL_MISSION;
  }
  if (strcmp(token, "FULL_MISSION") == 0 || strcmp(token, "FULL") == 0) {
    return TASK_FULL_MISSION;
  }
  if (strcmp(token, "PICKUP_MISSION") == 0 || strcmp(token, "PICKUP") == 0) {
    return TASK_PICKUP_MISSION;
  }
  if (strcmp(token, "DELIVERY_MISSION") == 0 || strcmp(token, "DELIVERY") == 0) {
    return TASK_DELIVERY_MISSION;
  }
  if (strcmp(token, "MANUAL_TEST") == 0 || strcmp(token, "MANUAL") == 0) {
    return TASK_MANUAL_TEST;
  }
  return TASK_NONE;
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
    case STAGE_IDLE:
      return "IDLE";
    case STAGE_START:
      return "START";
    case STAGE_TRAVEL_TO_PICKUP:
      return "TRAVEL_TO_PICKUP";
    case STAGE_VERIFY_PICKUP_STATION:
      return "VERIFY_PICKUP_STATION";
    case STAGE_PICKUP_SEQUENCE:
      return "PICKUP_SEQUENCE";
    case STAGE_OPEN_DOOR_FOR_LOAD:
      return "OPEN_DOOR_FOR_LOAD";
    case STAGE_LOAD_CARGO:
      return "LOAD_CARGO";
    case STAGE_CLOSE_DOOR_AFTER_LOAD:
      return "CLOSE_DOOR_AFTER_LOAD";
    case STAGE_TURN_AROUND:
      return "TURN_AROUND";
    case STAGE_TRAVEL_TO_DELIVERY:
      return "TRAVEL_TO_DELIVERY";
    case STAGE_VERIFY_DELIVERY_STATION:
      return "VERIFY_DELIVERY_STATION";
    case STAGE_WAIT_USER_UNLOCK:
      return "WAIT_USER_UNLOCK";
    case STAGE_OPEN_DOOR_FOR_UNLOAD:
      return "OPEN_DOOR_FOR_UNLOAD";
    case STAGE_WAIT_DOOR_CLOSE:
      return "WAIT_DOOR_CLOSE";
    case STAGE_AUTO_CLOSE_DOOR:
      return "AUTO_CLOSE_DOOR";
    case STAGE_REPORT_STATUS:
      return "REPORT_STATUS";
    case STAGE_DONE:
      return "DONE";
    case STAGE_FAULT:
      return "FAULT";
    case STAGE_NONE:
    default:
      return "";
  }
}

const char *savedStageName() {
  return autoStageName(savedAutoStage);
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
    case DRIVE_TURN_LEFT:
      return "TURN_LEFT";
    case DRIVE_TURN_RIGHT:
      return "TURN_RIGHT";
    case DRIVE_STOP:
    default:
      return "STOP";
  }
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

bool usesHeadingHold(DriveDirection direction) {
  return direction == DRIVE_FORWARD ||
         direction == DRIVE_BACKWARD ||
         direction == DRIVE_LEFT ||
         direction == DRIVE_RIGHT;
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

  int maxCorrection = min(HEADING_HOLD_MAX_PWM, max(12, motorPwm / 3));
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

  if (usesHeadingHold(currentDirection) &&
      (refreshHeadingTarget || previousDirection == DRIVE_STOP || previousDirection != currentDirection || !usesHeadingHold(previousDirection))) {
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

  int correctionPwm = usesHeadingHold(currentDirection) ? computeHeadingCorrectionPwm() : 0;
  lastHeadingCorrectionPwm = correctionPwm;
  const DirectionTrim &trim = DEFAULT_DIRECTION_TRIM[currentDirection];

  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    int basePwm = constrain(motorPwm + trim.offset[motor], MIN_MOTOR_PWM, MAX_MOTOR_PWM);
    int signedPwm = DRIVE_SIGN[currentDirection][motor] * basePwm - correctionPwm;
    applySignedMotorPwm(static_cast<MotorIndex>(motor), signedPwm);
  }
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

void setupTaskStatusLeds() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  redLedOn = false;
  greenLedOn = false;
}

void applyArmCommandInternal(const char *joint, int requestedDeg) {
  int angle = constrain(requestedDeg, MIN_ARM_DEG, MAX_ARM_DEG);
  if (strcmp(joint, "LIFT") == 0) {
    currentArmLiftDeg = angle;
    armLiftServo.write(currentArmLiftDeg);
  } else {
    currentArmGripDeg = angle;
    armGripServo.write(currentArmGripDeg);
  }
  lastArmCommandMs = millis();
}

void setupArm() {
  armLiftServo.attach(ARM_LIFT_SERVO_PIN);
  armGripServo.attach(ARM_GRIP_SERVO_PIN);
  applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
  applyArmCommandInternal("GRIP", DEFAULT_ARM_GRIP_DEG);
}

void setCargoDoorOpen(bool openDoor) {
  cargoDoorOpen = openDoor;
  cargoDoorServo.write(openDoor ? CARGO_DOOR_OPEN_DEG : CARGO_DOOR_CLOSED_DEG);
}

void setupCargoDoorServo() {
  cargoDoorServo.attach(CARGO_DOOR_SERVO_PIN);
  setCargoDoorOpen(false);
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

void Write_MFRC522(uchar addr, uchar val) {
  digitalWrite(RFID_SS_PIN, LOW);
  SPI.transfer((addr << 1) & 0x7E);
  SPI.transfer(val);
  digitalWrite(RFID_SS_PIN, HIGH);
}

uchar Read_MFRC522(uchar addr) {
  uchar val;
  digitalWrite(RFID_SS_PIN, LOW);
  SPI.transfer(((addr << 1) & 0x7E) | 0x80);
  val = SPI.transfer(0x00);
  digitalWrite(RFID_SS_PIN, HIGH);
  return val;
}

void SetBitMask(uchar reg, uchar mask) {
  uchar tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp | mask);
}

void ClearBitMask(uchar reg, uchar mask) {
  uchar tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp & (~mask));
}

void AntennaOn() {
  uchar temp = Read_MFRC522(TxControlReg);
  if (!(temp & 0x03)) {
    SetBitMask(TxControlReg, 0x03);
  }
}

void MFRC522_Reset() {
  Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

void MFRC522_Init() {
  digitalWrite(RFID_RST_PIN, HIGH);
  MFRC522_Reset();
  Write_MFRC522(TModeReg, 0x8D);
  Write_MFRC522(TPrescalerReg, 0x3E);
  Write_MFRC522(TReloadRegL, 30);
  Write_MFRC522(TReloadRegH, 0);
  Write_MFRC522(TxAutoReg, 0x40);
  Write_MFRC522(ModeReg, 0x3D);
  AntennaOn();
}

uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen) {
  uchar status = MI_ERR;
  uchar irqEn = 0x00;
  uchar waitIRq = 0x00;
  uchar lastBits;
  uchar n;
  uint i;

  switch (command) {
    case PCD_TRANSCEIVE:
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    default:
      break;
  }

  Write_MFRC522(CommIEnReg, irqEn | 0x80);
  ClearBitMask(CommIrqReg, 0x80);
  SetBitMask(FIFOLevelReg, 0x80);
  Write_MFRC522(CommandReg, PCD_IDLE);

  for (i = 0; i < sendLen; i++) {
    Write_MFRC522(FIFODataReg, sendData[i]);
  }

  Write_MFRC522(CommandReg, command);
  if (command == PCD_TRANSCEIVE) {
    SetBitMask(BitFramingReg, 0x80);
  }

  i = 2000;
  do {
    n = Read_MFRC522(CommIrqReg);
    i--;
  } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

  ClearBitMask(BitFramingReg, 0x80);

  if (i != 0) {
    if (!(Read_MFRC522(ErrorReg) & 0x1B)) {
      status = MI_OK;
      if (n & irqEn & 0x01) {
        status = MI_NOTAGERR;
      }

      if (command == PCD_TRANSCEIVE) {
        n = Read_MFRC522(FIFOLevelReg);
        lastBits = Read_MFRC522(ControlReg) & 0x07;
        if (lastBits) {
          *backLen = (n - 1) * 8 + lastBits;
        } else {
          *backLen = n * 8;
        }

        if (n == 0) {
          n = 1;
        }
        if (n > MAX_LEN) {
          n = MAX_LEN;
        }

        for (i = 0; i < n; i++) {
          backData[i] = Read_MFRC522(FIFODataReg);
        }
      }
    }
  }

  return status;
}

uchar MFRC522_Request(uchar reqMode, uchar *tagType) {
  uchar status;
  uint backBits;

  Write_MFRC522(BitFramingReg, 0x07);
  tagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10)) {
    status = MI_ERR;
  }

  return status;
}

uchar MFRC522_Anticoll(uchar *serNum) {
  uchar status;
  uchar i;
  uchar serNumCheck = 0;
  uint unLen;

  Write_MFRC522(BitFramingReg, 0x00);
  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == MI_OK) {
    for (i = 0; i < 4; i++) {
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i]) {
      status = MI_ERR;
    }
  }

  return status;
}

void setupRfidReader() {
  pinMode(RFID_SS_PIN, OUTPUT);
  digitalWrite(RFID_SS_PIN, HIGH);
  pinMode(RFID_RST_PIN, OUTPUT);
  digitalWrite(RFID_RST_PIN, HIGH);
  SPI.begin();
  MFRC522_Init();
}

void formatUidHex(const uchar *uid, size_t count, char *output, size_t outputSize) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  size_t outIndex = 0;

  if (outputSize == 0) {
    return;
  }

  for (size_t i = 0; i < count && outIndex + 2 < outputSize; ++i) {
    output[outIndex++] = HEX_DIGITS[(uid[i] >> 4) & 0x0F];
    output[outIndex++] = HEX_DIGITS[uid[i] & 0x0F];
  }
  output[outIndex] = '\0';
}

void recordRfidObservation(const char *uidText) {
  copyText(lastObservedUid, sizeof(lastObservedUid), uidText);
  copyText(latestRfidTag, sizeof(latestRfidTag), uidText);

  pickupRfidMatched = strcmp(uidText, PICKUP_RFID_UID) == 0;
  deliveryRfidMatched = strcmp(uidText, DELIVERY_RFID_UID) == 0;

  if (pickupRfidMatched) {
    copyText(lastRfidStation, sizeof(lastRfidStation), "PICKUP");
  } else if (deliveryRfidMatched) {
    copyText(lastRfidStation, sizeof(lastRfidStation), "DELIVERY");
  } else {
    copyText(lastRfidStation, sizeof(lastRfidStation), "UNKNOWN");
  }
}

void updateRfidReader() {
  if (millis() - lastRfidPollMs < RFID_POLL_INTERVAL_MS) {
    return;
  }
  lastRfidPollMs = millis();

  uchar cardType[2];
  uchar uid[5];
  uchar status = MFRC522_Request(PICC_REQIDL, cardType);
  if (status != MI_OK) {
    lastRfidCardPresent = false;
    return;
  }

  status = MFRC522_Anticoll(uid);
  if (status != MI_OK) {
    lastRfidCardPresent = false;
    return;
  }

  char uidHex[24];
  formatUidHex(uid, 4, uidHex, sizeof(uidHex));
  if (!lastRfidCardPresent || strcmp(uidHex, lastObservedUid) != 0) {
    recordRfidObservation(uidHex);
  }
  lastRfidCardPresent = true;
}

bool currentStageIsPickupSide() {
  return currentAutoStage == STAGE_TRAVEL_TO_PICKUP ||
         currentAutoStage == STAGE_VERIFY_PICKUP_STATION ||
         currentAutoStage == STAGE_PICKUP_SEQUENCE ||
         currentAutoStage == STAGE_OPEN_DOOR_FOR_LOAD ||
         currentAutoStage == STAGE_LOAD_CARGO ||
         currentAutoStage == STAGE_CLOSE_DOOR_AFTER_LOAD;
}

bool stageRfidMatchedForCurrentStage() {
  if (currentStageIsPickupSide()) {
    return pickupRfidMatched;
  }
  if (currentAutoStage == STAGE_TRAVEL_TO_DELIVERY || currentAutoStage == STAGE_VERIFY_DELIVERY_STATION) {
    return deliveryRfidMatched;
  }
  return false;
}

bool shouldTrackObstacleInCurrentStage() {
  return currentAutoStage == STAGE_TRAVEL_TO_PICKUP || currentAutoStage == STAGE_TRAVEL_TO_DELIVERY;
}

void updateTaskStatusLeds() {
  bool nextRed = obstacleRoutineActive || currentMode == MODE_FAULT || faultLatched;
  bool nextGreen = (currentMode == MODE_AUTO && currentAutoStage == STAGE_WAIT_USER_UNLOCK);

  if (nextRed != redLedOn) {
    digitalWrite(RED_LED_PIN, nextRed ? HIGH : LOW);
    redLedOn = nextRed;
  }
  if (nextGreen != greenLedOn) {
    digitalWrite(GREEN_LED_PIN, nextGreen ? HIGH : LOW);
    greenLedOn = nextGreen;
  }
}

const char *cargoDoorState() {
  return cargoDoorOpen ? "OPEN" : "CLOSED";
}

const char *armStatus() {
  if (lastArmCommandMs > 0 && millis() - lastArmCommandMs < ARM_ACTIVE_WINDOW_MS) {
    return "ACTIVE";
  }
  return "IDLE";
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
      case DRIVE_TURN_LEFT:
        return "MANUAL_TURN_LEFT";
      case DRIVE_TURN_RIGHT:
        return "MANUAL_TURN_RIGHT";
      case DRIVE_STOP:
      default:
        return "MANUAL_OVERRIDE";
    }
  }

  if (currentMode == MODE_FAULT) {
    return "FAULT";
  }

  const char *autoName = autoStageName(currentAutoStage);
  if (autoName[0] != '\0') {
    return autoName;
  }

  return "IDLE";
}

const char *deliveryResult() {
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    if (currentTask == TASK_MANUAL_TEST && taskActive) {
      return "MANUAL_TEST";
    }
    return resumePending ? "RESUME_PENDING" : "MANUAL_ONLY";
  }

  if (currentMode == MODE_FAULT || faultLatched) {
    return "FAULT";
  }

  if (currentAutoStage == STAGE_VERIFY_PICKUP_STATION || currentAutoStage == STAGE_VERIFY_DELIVERY_STATION) {
    return stageRfidMatchedForCurrentStage() ? "VERIFY_OK" : "VERIFYING";
  }

  if (currentAutoStage == STAGE_WAIT_USER_UNLOCK) {
    return "WAITING_UNLOCK";
  }

  if (currentAutoStage == STAGE_WAIT_DOOR_CLOSE || currentAutoStage == STAGE_AUTO_CLOSE_DOOR) {
    return "UNLOAD_OPEN";
  }

  if (currentAutoStage == STAGE_DONE) {
    return "TASK_DONE";
  }

  if (taskActive) {
    return "TASK_RUNNING";
  }

  if (cargoDoorOpen) {
    return "UNLOCKED";
  }

  return "READY";
}

AutoStage safeResumeStageFor(AutoStage stage) {
  switch (stage) {
    case STAGE_START:
    case STAGE_TRAVEL_TO_PICKUP:
    case STAGE_VERIFY_PICKUP_STATION:
    case STAGE_PICKUP_SEQUENCE:
    case STAGE_OPEN_DOOR_FOR_LOAD:
    case STAGE_LOAD_CARGO:
    case STAGE_CLOSE_DOOR_AFTER_LOAD:
    case STAGE_TURN_AROUND:
    case STAGE_TRAVEL_TO_DELIVERY:
    case STAGE_VERIFY_DELIVERY_STATION:
    case STAGE_WAIT_USER_UNLOCK:
    case STAGE_OPEN_DOOR_FOR_UNLOAD:
    case STAGE_WAIT_DOOR_CLOSE:
    case STAGE_AUTO_CLOSE_DOOR:
    case STAGE_REPORT_STATUS:
      return stage;
    default:
      return STAGE_NONE;
  }
}

void resetStageLocalState() {
  invalidDistanceCount = 0;
  dockStableStartedMs = 0;
  pickupSequenceStep = 0;
  armStepStartedMs = 0;
  unlockApproved = false;
  doorManualCloseRequested = false;
  obstacleRoutineActive = false;
  obstacleRoutineStep = 0;
  obstacleStepStartedMs = 0;
}

void enterAutoStage(AutoStage stage) {
  currentAutoStage = stage;
  autoStageStartedMs = millis();
  resetStageLocalState();

  switch (stage) {
    case STAGE_IDLE:
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;
    case STAGE_START:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_TRAVEL_PWM, false);
      captureHeadingTarget();
      setStatusNote("Mission start stage");
      break;
    case STAGE_TRAVEL_TO_PICKUP:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setStatusNote("Traveling to pickup station");
      break;
    case STAGE_VERIFY_PICKUP_STATION:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Verifying pickup RFID");
      break;
    case STAGE_PICKUP_SEQUENCE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Running pickup sequence");
      break;
    case STAGE_OPEN_DOOR_FOR_LOAD:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Opening door for load");
      break;
    case STAGE_LOAD_CARGO:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Waiting for cargo load");
      break;
    case STAGE_CLOSE_DOOR_AFTER_LOAD:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Closing door after load");
      break;
    case STAGE_TURN_AROUND:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setStatusNote("Turning around");
      break;
    case STAGE_TRAVEL_TO_DELIVERY:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setStatusNote("Traveling to delivery station");
      break;
    case STAGE_VERIFY_DELIVERY_STATION:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Verifying delivery RFID");
      break;
    case STAGE_WAIT_USER_UNLOCK:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Waiting for Jetson unlock");
      break;
    case STAGE_OPEN_DOOR_FOR_UNLOAD:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Opening door for unload");
      break;
    case STAGE_WAIT_DOOR_CLOSE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Waiting for door close");
      break;
    case STAGE_AUTO_CLOSE_DOOR:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Auto-closing cargo door");
      break;
    case STAGE_REPORT_STATUS:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = true;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Reporting mission result");
      break;
    case STAGE_DONE:
      currentMode = MODE_AUTO;
      manualSessionActive = false;
      taskActive = false;
      clearResumeState();
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      setStatusNote("Mission complete");
      break;
    case STAGE_FAULT:
      currentMode = MODE_FAULT;
      manualSessionActive = false;
      taskActive = false;
      stopCar();
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;
    case STAGE_NONE:
    default:
      break;
  }
}

void clearTaskContext() {
  currentTask = TASK_NONE;
  taskActive = false;
  manualSessionActive = false;
  clearResumeState();
}

void enterFault(const char *message) {
  stopCar();
  setMotionCommand(DRIVE_STOP, motorPwm, false);
  currentMode = MODE_FAULT;
  currentAutoStage = STAGE_FAULT;
  taskActive = false;
  resumePending = false;
  savedAutoStage = STAGE_NONE;
  manualSessionActive = false;
  faultLatched = true;
  obstacleRoutineActive = false;
  obstacleRoutineStep = 0;
  updateTaskStatusLeds();
  setFaultText(message);
  setResumeNote("Resume blocked by fault");
  setStatusNote("Mission faulted");
}

bool startTaskInternal(MissionTask task) {
  if (task == TASK_NONE) {
    setStatusNote("Unknown task ID");
    return false;
  }

  if (currentMode == MODE_MANUAL_OVERRIDE &&
      (resumePending || taskActive || savedAutoStage != STAGE_NONE || currentTask != TASK_NONE)) {
    setStatusNote("Start rejected while manual holds a task");
    return false;
  }

  clearFaultText();
  faultLatched = false;
  commandTimedOut = false;
  clearResumeState();
  clearRfidState();
  doorAutoClosed = false;
  doorManualCloseRequested = false;
  unlockApproved = false;
  unloadDoorOpenedMs = 0;
  setCargoDoorOpen(false);
  applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
  applyArmCommandInternal("GRIP", DEFAULT_ARM_GRIP_DEG);

  currentTask = task;

  if (task == TASK_MANUAL_TEST) {
    currentMode = MODE_MANUAL_OVERRIDE;
    manualSessionActive = true;
    taskActive = true;
    currentAutoStage = STAGE_IDLE;
    setMotionCommand(DRIVE_STOP, motorPwm, false);
    setStatusNote("Manual test active");
    return true;
  }

  if (!mpuReady) {
    enterFault("MPU6050 not ready");
    return false;
  }

  enterAutoStage(STAGE_START);
  return true;
}

void abortTaskToIdle() {
  clearFaultText();
  faultLatched = false;
  clearRfidState();
  clearTaskContext();
  currentMode = MODE_AUTO;
  setCargoDoorOpen(false);
  unloadDoorOpenedMs = 0;
  enterAutoStage(STAGE_IDLE);
  setStatusNote("Task aborted to idle");
}

void enterManualOverride() {
  if (currentMode == MODE_MANUAL_OVERRIDE) {
    setStatusNote("Manual override already active");
    return;
  }

  AutoStage safeStage = STAGE_NONE;
  if (taskActive && currentTask != TASK_MANUAL_TEST) {
    safeStage = safeResumeStageFor(currentAutoStage);
  }

  if (safeStage != STAGE_NONE) {
    savedAutoStage = safeStage;
    resumePending = true;
    setResumeNote(autoStageName(safeStage));
  } else {
    clearResumeState();
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
    setStatusNote("Manual override active");
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

  if (!resumePending || savedAutoStage == STAGE_NONE || currentTask == TASK_MANUAL_TEST) {
    setStatusNote("Resume rejected, no saved stage");
    return false;
  }

  AutoStage stageToResume = savedAutoStage;
  clearResumeState();
  manualSessionActive = false;
  enterAutoStage(stageToResume);
  setStatusNote("Resumed from saved stage entry");
  return true;
}

void abortResumeWhileManual() {
  stopCar();
  setMotionCommand(DRIVE_STOP, motorPwm, false);
  clearResumeState();
  clearRfidState();
  currentTask = TASK_NONE;
  taskActive = false;
  currentAutoStage = STAGE_IDLE;
  autoStageStartedMs = millis();
  manualSessionActive = true;
  currentMode = MODE_MANUAL_OVERRIDE;
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
  if (strcmp(token, "TURN_LEFT") == 0) {
    *ok = true;
    return DRIVE_TURN_LEFT;
  }
  if (strcmp(token, "TURN_RIGHT") == 0) {
    *ok = true;
    return DRIVE_TURN_RIGHT;
  }
  if (strcmp(token, "STOP") == 0) {
    *ok = true;
    return DRIVE_STOP;
  }

  *ok = false;
  return DRIVE_STOP;
}

bool shouldStartObstacleRoutine() {
  return currentMode == MODE_AUTO &&
         shouldTrackObstacleInCurrentStage() &&
         !obstacleRoutineActive &&
         frontDistanceValid() &&
         latestAverageCm > 0.0f &&
         latestAverageCm < OBSTACLE_TRIGGER_CM &&
         !stageRfidMatchedForCurrentStage();
}

void advanceObstacleRoutine() {
  obstacleRoutineStep++;
  obstacleStepStartedMs = millis();
}

void startObstacleRoutine() {
  obstacleRoutineActive = true;
  obstacleRoutineStep = 0;
  obstacleStepStartedMs = millis();
  stopCar();
  setStatusNote("Obstacle routine active");
}

void updateObstacleRoutine() {
  unsigned long elapsedMs = millis() - obstacleStepStartedMs;

  switch (obstacleRoutineStep) {
    case 0:
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      if (elapsedMs >= OBSTACLE_STOP_HOLD_MS) {
        advanceObstacleRoutine();
      }
      break;
    case 1:
      setMotionCommand(DRIVE_RIGHT, AUTO_STRAFE_PWM, false);
      if (elapsedMs >= OBSTACLE_STRAFE_MS) {
        advanceObstacleRoutine();
      }
      break;
    case 2:
      setMotionCommand(DRIVE_FORWARD, AUTO_APPROACH_PWM, true);
      if (elapsedMs >= OBSTACLE_PASS_MS) {
        advanceObstacleRoutine();
      }
      break;
    case 3:
      setMotionCommand(DRIVE_LEFT, AUTO_STRAFE_PWM, false);
      if (elapsedMs >= OBSTACLE_RETURN_MS) {
        advanceObstacleRoutine();
      }
      break;
    default:
      obstacleRoutineActive = false;
      obstacleRoutineStep = 0;
      setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
      setStatusNote("Obstacle routine cleared");
      break;
  }
}

void runTravelStage(bool pickupPhase) {
  if (pickupPhase ? pickupRfidMatched : deliveryRfidMatched) {
    setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
    enterAutoStage(pickupPhase ? STAGE_VERIFY_PICKUP_STATION : STAGE_VERIFY_DELIVERY_STATION);
    return;
  }

  if (!requireFrontDistance("Front distance invalid during travel")) {
    return;
  }

  if (fabs(latestDiffCm) > DOCK_ALIGN_TOLERANCE_CM && latestAverageCm <= APPROACH_TO_DOCK_CM) {
    if (latestDiffCm > 0.0f) {
      setMotionCommand(DRIVE_RIGHT, AUTO_STRAFE_PWM, false);
      setStatusNote(pickupPhase ? "Pickup align strafing right" : "Delivery align strafing right");
    } else {
      setMotionCommand(DRIVE_LEFT, AUTO_STRAFE_PWM, false);
      setStatusNote(pickupPhase ? "Pickup align strafing left" : "Delivery align strafing left");
    }
    dockStableStartedMs = 0;
    return;
  }

  if (latestAverageCm > APPROACH_TO_DOCK_CM) {
    setMotionCommand(DRIVE_FORWARD, AUTO_TRAVEL_PWM, false);
    setStatusNote(pickupPhase ? "Traveling to pickup RFID" : "Traveling to delivery RFID");
    dockStableStartedMs = 0;
    return;
  }

  if (latestAverageCm > DOCK_TARGET_CM + DOCK_DISTANCE_TOLERANCE_CM) {
    setMotionCommand(DRIVE_FORWARD, AUTO_APPROACH_PWM, false);
    setStatusNote(pickupPhase ? "Approaching pickup dock" : "Approaching delivery dock");
    dockStableStartedMs = 0;
    return;
  }

  if (latestAverageCm < DOCK_TARGET_CM - DOCK_DISTANCE_TOLERANCE_CM) {
    setMotionCommand(DRIVE_BACKWARD, AUTO_DOCK_PWM, false);
    setStatusNote(pickupPhase ? "Backing off pickup dock" : "Backing off delivery dock");
    dockStableStartedMs = 0;
    return;
  }

  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (dockStableStartedMs == 0) {
    dockStableStartedMs = millis();
    setStatusNote(pickupPhase ? "Holding at pickup dock" : "Holding at delivery dock");
    return;
  }

  if (millis() - dockStableStartedMs >= DOCK_STABLE_MS) {
    enterAutoStage(pickupPhase ? STAGE_VERIFY_PICKUP_STATION : STAGE_VERIFY_DELIVERY_STATION);
  }
}

void runVerifyStage(bool pickupPhase) {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  bool matched = pickupPhase ? pickupRfidMatched : deliveryRfidMatched;
  if (matched && millis() - autoStageStartedMs >= VERIFY_SETTLE_MS) {
    setStatusNote(pickupPhase ? "Pickup RFID verified" : "Delivery RFID verified");
    enterAutoStage(pickupPhase ? STAGE_PICKUP_SEQUENCE : STAGE_WAIT_USER_UNLOCK);
    return;
  }

  if (millis() - autoStageStartedMs >= VERIFY_TIMEOUT_MS) {
    enterFault(pickupPhase ? "Pickup RFID verify timeout" : "Delivery RFID verify timeout");
  }
}

void runPickupSequenceStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);

  switch (pickupSequenceStep) {
    case 0:
      applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
      armStepStartedMs = millis();
      pickupSequenceStep = 1;
      break;
    case 1:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("GRIP", ARM_GRIP_OPEN_DEG);
        armStepStartedMs = millis();
        pickupSequenceStep = 2;
      }
      break;
    case 2:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("LIFT", ARM_LIFT_PICK_DEG);
        armStepStartedMs = millis();
        pickupSequenceStep = 3;
      }
      break;
    case 3:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("GRIP", ARM_GRIP_CLOSE_DEG);
        armStepStartedMs = millis();
        pickupSequenceStep = 4;
      }
      break;
    case 4:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("LIFT", DEFAULT_ARM_LIFT_DEG);
        armStepStartedMs = millis();
        pickupSequenceStep = 5;
      }
      break;
    default:
      if (millis() - armStepStartedMs >= ARM_STEP_WAIT_MS) {
        applyArmCommandInternal("GRIP", DEFAULT_ARM_GRIP_DEG);
        enterAutoStage(STAGE_OPEN_DOOR_FOR_LOAD);
      }
      break;
  }
}

void runOpenDoorForLoadStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs < 20UL) {
    setCargoDoorOpen(true);
  }
  if (millis() - autoStageStartedMs >= DOOR_SETTLE_MS) {
    enterAutoStage(STAGE_LOAD_CARGO);
  }
}

void runLoadCargoStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs >= LOAD_SETTLE_MS) {
    enterAutoStage(STAGE_CLOSE_DOOR_AFTER_LOAD);
  }
}

void runCloseDoorAfterLoadStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs < 20UL) {
    setCargoDoorOpen(false);
  }
  if (millis() - autoStageStartedMs >= DOOR_SETTLE_MS) {
    if (currentTask == TASK_PICKUP_MISSION) {
      enterAutoStage(STAGE_REPORT_STATUS);
    } else {
      enterAutoStage(STAGE_TURN_AROUND);
    }
  }
}

void runTurnAroundStage() {
  setMotionCommand(DRIVE_TURN_RIGHT, AUTO_TURN_PWM, false);
  if (millis() - autoStageStartedMs >= TURN_AROUND_MS) {
    setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
    enterAutoStage(STAGE_TRAVEL_TO_DELIVERY);
  }
}

void runWaitUserUnlockStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (unlockApproved) {
    unlockApproved = false;
    enterAutoStage(STAGE_OPEN_DOOR_FOR_UNLOAD);
  }
}

void runOpenDoorForUnloadStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs < 20UL) {
    setCargoDoorOpen(true);
    unloadDoorOpenedMs = millis();
    doorManualCloseRequested = false;
    doorAutoClosed = false;
  }
  if (millis() - autoStageStartedMs >= DOOR_SETTLE_MS) {
    enterAutoStage(STAGE_WAIT_DOOR_CLOSE);
  }
}

void runWaitDoorCloseStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (doorManualCloseRequested) {
    setCargoDoorOpen(false);
    doorManualCloseRequested = false;
    enterAutoStage(STAGE_REPORT_STATUS);
    return;
  }

  if (unloadDoorOpenedMs > 0 && millis() - unloadDoorOpenedMs >= DELIVERY_DOOR_AUTO_CLOSE_MS) {
    enterAutoStage(STAGE_AUTO_CLOSE_DOOR);
  }
}

void runAutoCloseDoorStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs < 20UL) {
    setCargoDoorOpen(false);
    doorAutoClosed = true;
  }
  if (millis() - autoStageStartedMs >= DOOR_SETTLE_MS) {
    enterAutoStage(STAGE_REPORT_STATUS);
  }
}

void runReportStatusStage() {
  setMotionCommand(DRIVE_STOP, AUTO_DOCK_PWM, false);
  if (millis() - autoStageStartedMs >= REPORT_STATUS_MS) {
    enterAutoStage(STAGE_DONE);
  }
}

void runMissionStateMachine() {
  if (currentMode != MODE_AUTO || currentTask == TASK_NONE || currentTask == TASK_MANUAL_TEST) {
    return;
  }

  if (shouldStartObstacleRoutine()) {
    startObstacleRoutine();
  }

  if (obstacleRoutineActive) {
    updateObstacleRoutine();
    return;
  }

  switch (currentAutoStage) {
    case STAGE_IDLE:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;
    case STAGE_START:
      setMotionCommand(DRIVE_STOP, AUTO_TRAVEL_PWM, false);
      if (millis() - autoStageStartedMs >= START_SETTLE_MS) {
        if (currentTask == TASK_DELIVERY_MISSION) {
          enterAutoStage(STAGE_TRAVEL_TO_DELIVERY);
        } else {
          enterAutoStage(STAGE_TRAVEL_TO_PICKUP);
        }
      }
      break;
    case STAGE_TRAVEL_TO_PICKUP:
      runTravelStage(true);
      break;
    case STAGE_VERIFY_PICKUP_STATION:
      runVerifyStage(true);
      break;
    case STAGE_PICKUP_SEQUENCE:
      runPickupSequenceStage();
      break;
    case STAGE_OPEN_DOOR_FOR_LOAD:
      runOpenDoorForLoadStage();
      break;
    case STAGE_LOAD_CARGO:
      runLoadCargoStage();
      break;
    case STAGE_CLOSE_DOOR_AFTER_LOAD:
      runCloseDoorAfterLoadStage();
      break;
    case STAGE_TURN_AROUND:
      runTurnAroundStage();
      break;
    case STAGE_TRAVEL_TO_DELIVERY:
      runTravelStage(false);
      break;
    case STAGE_VERIFY_DELIVERY_STATION:
      runVerifyStage(false);
      break;
    case STAGE_WAIT_USER_UNLOCK:
      runWaitUserUnlockStage();
      break;
    case STAGE_OPEN_DOOR_FOR_UNLOAD:
      runOpenDoorForUnloadStage();
      break;
    case STAGE_WAIT_DOOR_CLOSE:
      runWaitDoorCloseStage();
      break;
    case STAGE_AUTO_CLOSE_DOOR:
      runAutoCloseDoorStage();
      break;
    case STAGE_REPORT_STATUS:
      runReportStatusStage();
      break;
    case STAGE_DONE:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;
    case STAGE_FAULT:
      setMotionCommand(DRIVE_STOP, motorPwm, false);
      break;
    case STAGE_NONE:
    default:
      break;
  }
}

void printJsonEscaped(const char *value) {
  Serial.print('"');
  if (value != NULL) {
    for (const char *cursor = value; *cursor != '\0'; ++cursor) {
      char ch = *cursor;
      if (ch == '"' || ch == '\\') {
        Serial.print('\\');
        Serial.print(ch);
      } else if (ch == '\n') {
        Serial.print(F("\\n"));
      } else if (ch == '\r') {
        Serial.print(F("\\r"));
      } else if (ch == '\t') {
        Serial.print(F("\\t"));
      } else {
        Serial.print(ch);
      }
    }
  }
  Serial.print('"');
}

void printJsonBool(bool value) {
  Serial.print(value ? F("true") : F("false"));
}

void printJsonFloat(float value, uint8_t precision) {
  if (value < 0.0f) {
    Serial.print(F("null"));
    return;
  }
  Serial.print(value, precision);
}

void printJsonFloatOrNull(float value, uint8_t precision, bool available) {
  if (!available) {
    Serial.print(F("null"));
    return;
  }
  Serial.print(value, precision);
}

void emitTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = millis();

  bool anyRfidMatch = pickupRfidMatched || deliveryRfidMatched;

  Serial.print('{');
  Serial.print(F("\"ts_ms\":"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(F("\"mode_protocol_version\":"));
  printJsonEscaped(MODE_PROTOCOL_VERSION);
  Serial.print(',');
  Serial.print(F("\"state\":"));
  printJsonEscaped(stateName());
  Serial.print(',');
  Serial.print(F("\"current_mode\":"));
  printJsonEscaped(runModeName(currentMode));
  Serial.print(',');
  Serial.print(F("\"current_auto_stage\":"));
  printJsonEscaped(autoStageName(currentAutoStage));
  Serial.print(',');
  Serial.print(F("\"saved_auto_stage\":"));
  printJsonEscaped(savedStageName());
  Serial.print(',');
  Serial.print(F("\"resume_pending\":"));
  printJsonBool(resumePending);
  Serial.print(',');
  Serial.print(F("\"manual_session_active\":"));
  printJsonBool(manualSessionActive);
  Serial.print(',');
  Serial.print(F("\"task_active\":"));
  printJsonBool(taskActive);
  Serial.print(',');
  Serial.print(F("\"fault_latched\":"));
  printJsonBool(faultLatched);
  Serial.print(',');
  Serial.print(F("\"resume_note\":"));
  printJsonEscaped(resumeNote);
  Serial.print(',');
  Serial.print(F("\"status_note\":"));
  printJsonEscaped(statusNote);
  Serial.print(',');
  Serial.print(F("\"selected_task\":"));
  printJsonEscaped(taskName(currentTask));
  Serial.print(',');
  Serial.print(F("\"cargo_door\":"));
  printJsonEscaped(cargoDoorState());
  Serial.print(',');
  Serial.print(F("\"door_auto_close_ms\":"));
  Serial.print(DELIVERY_DOOR_AUTO_CLOSE_MS);
  Serial.print(',');
  Serial.print(F("\"pickup_rfid_matched\":"));
  printJsonBool(pickupRfidMatched);
  Serial.print(',');
  Serial.print(F("\"delivery_rfid_matched\":"));
  printJsonBool(deliveryRfidMatched);
  Serial.print(',');
  Serial.print(F("\"last_rfid_station\":"));
  printJsonEscaped(lastRfidStation);
  Serial.print(',');
  Serial.print(F("\"rfid_tag\":"));
  printJsonEscaped(latestRfidTag);
  Serial.print(',');
  Serial.print(F("\"rfid_match\":"));
  printJsonBool(anyRfidMatch);
  Serial.print(',');
  Serial.print(F("\"obstacle_routine_active\":"));
  printJsonBool(obstacleRoutineActive);
  Serial.print(',');
  Serial.print(F("\"obstacle_routine_step\":"));
  Serial.print(obstacleRoutineActive ? static_cast<int>(obstacleRoutineStep) : -1);
  Serial.print(',');
  Serial.print(F("\"red_led\":"));
  printJsonBool(redLedOn);
  Serial.print(',');
  Serial.print(F("\"green_led\":"));
  printJsonBool(greenLedOn);
  Serial.print(',');
  Serial.print(F("\"control_direction\":"));
  printJsonEscaped(directionName(currentDirection));
  Serial.print(',');
  Serial.print(F("\"motor_pwm\":"));
  Serial.print(motorPwm);
  Serial.print(',');
  Serial.print(F("\"motor_pwm_a\":"));
  Serial.print(currentWheelPwm[MOTOR_A]);
  Serial.print(',');
  Serial.print(F("\"motor_pwm_b\":"));
  Serial.print(currentWheelPwm[MOTOR_B]);
  Serial.print(',');
  Serial.print(F("\"motor_pwm_c\":"));
  Serial.print(currentWheelPwm[MOTOR_C]);
  Serial.print(',');
  Serial.print(F("\"motor_pwm_d\":"));
  Serial.print(currentWheelPwm[MOTOR_D]);
  Serial.print(',');
  Serial.print(F("\"control_status\":"));
  printJsonEscaped(controlStatus());
  Serial.print(',');
  Serial.print(F("\"arm_lift_deg\":"));
  Serial.print(currentArmLiftDeg);
  Serial.print(',');
  Serial.print(F("\"arm_grip_deg\":"));
  Serial.print(currentArmGripDeg);
  Serial.print(',');
  Serial.print(F("\"arm_status\":"));
  printJsonEscaped(armStatus());
  Serial.print(',');
  Serial.print(F("\"right_cm\":"));
  printJsonFloat(latestDistances.rightCm, 1);
  Serial.print(',');
  Serial.print(F("\"left_cm\":"));
  printJsonFloat(latestDistances.leftCm, 1);
  Serial.print(',');
  Serial.print(F("\"avg_cm\":"));
  printJsonFloat(latestAverageCm, 1);
  Serial.print(',');
  Serial.print(F("\"diff_cm\":"));
  printJsonFloat(latestDiffCm, 1);
  Serial.print(',');
  Serial.print(F("\"heading_deg\":"));
  printJsonFloatOrNull(headingDeg, 2, mpuReady);
  Serial.print(',');
  Serial.print(F("\"heading_target_deg\":"));
  printJsonFloatOrNull(headingTargetDeg, 2, mpuReady);
  Serial.print(',');
  Serial.print(F("\"heading_error_deg\":"));
  printJsonFloatOrNull(headingTargetDeg - headingDeg, 2, mpuReady);
  Serial.print(',');
  Serial.print(F("\"gyro_z_dps\":"));
  printJsonFloatOrNull(latestGyroZDps, 2, mpuReady);
  Serial.print(',');
  Serial.print(F("\"heading_correction_pwm\":"));
  Serial.print(lastHeadingCorrectionPwm);
  Serial.print(',');
  Serial.print(F("\"command_age_ms\":"));
  if (lastDriveCommandMs == 0) {
    Serial.print(F("null"));
  } else {
    Serial.print(millis() - lastDriveCommandMs);
  }
  Serial.print(',');
  Serial.print(F("\"delivery_result\":"));
  printJsonEscaped(deliveryResult());
  Serial.print(',');
  Serial.print(F("\"fault\":"));
  printJsonEscaped(lastFault);
  Serial.println('}');
}

void handleOpenDoorCommand() {
  if (currentMode == MODE_AUTO && currentAutoStage == STAGE_WAIT_USER_UNLOCK) {
    unlockApproved = true;
    setStatusNote("Jetson unlock approved");
    return;
  }

  setCargoDoorOpen(true);
  if (currentMode == MODE_AUTO && currentAutoStage == STAGE_WAIT_DOOR_CLOSE) {
    unloadDoorOpenedMs = millis();
  }
  setStatusNote("Door opened by serial command");
}

void handleCloseDoorCommand() {
  if (currentMode == MODE_AUTO && currentAutoStage == STAGE_WAIT_DOOR_CLOSE && cargoDoorOpen) {
    doorManualCloseRequested = true;
    setStatusNote("Manual close requested");
    return;
  }

  setCargoDoorOpen(false);
  unloadDoorOpenedMs = 0;
  setStatusNote("Door closed by serial command");
}

void printHelp() {
  Serial.println(F("Final integrated AMR firmware ready."));
  Serial.println(F("Task commands:"));
  Serial.println(F("  TASK START FULL_MISSION"));
  Serial.println(F("  TASK START PICKUP_MISSION"));
  Serial.println(F("  TASK START DELIVERY_MISSION"));
  Serial.println(F("  TASK START MANUAL_TEST"));
  Serial.println(F("  TASK ABORT"));
  Serial.println(F("Override commands:"));
  Serial.println(F("  OVERRIDE TAKE"));
  Serial.println(F("  OVERRIDE RESUME"));
  Serial.println(F("  OVERRIDE ABORT"));
  Serial.println(F("Manual control:"));
  Serial.println(F("  DRIVE FORWARD <0-255>"));
  Serial.println(F("  DRIVE BACKWARD <0-255>"));
  Serial.println(F("  DRIVE LEFT <0-255>"));
  Serial.println(F("  DRIVE RIGHT <0-255>"));
  Serial.println(F("  DRIVE TURN_LEFT <0-255>"));
  Serial.println(F("  DRIVE TURN_RIGHT <0-255>"));
  Serial.println(F("  DRIVE STOP <0-255|0>"));
  Serial.println(F("  ARM LIFT <0-180>"));
  Serial.println(F("  ARM GRIP <0-180>"));
  Serial.println(F("Door / maintenance:"));
  Serial.println(F("  OPEN_DOOR"));
  Serial.println(F("  CLOSE_DOOR"));
  Serial.println(F("  GYRO"));
  Serial.println(F("  STATUS"));
  Serial.println(F("Legacy compatibility: START FULL|PICKUP|DELIVERY, ABORT"));
}

void handleCommand(const char *command) {
  char buffer[96];
  strncpy(buffer, command, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "OPEN_DOOR") == 0) {
    handleOpenDoorCommand();
    return;
  }

  if (strcmp(token, "CLOSE_DOOR") == 0) {
    handleCloseDoorCommand();
    return;
  }

  if (strcmp(token, "STATUS") == 0) {
    emitTelemetry();
    return;
  }

  if (strcmp(token, "HELP") == 0) {
    printHelp();
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

  if (strcmp(token, "START") == 0) {
    char *legacyTask = strtok(NULL, " ");
    MissionTask task = parseTaskId(legacyTask);
    if (task == TASK_NONE) {
      setStatusNote("Unknown legacy START task");
      return;
    }
    startTaskInternal(task);
    return;
  }

  if (strcmp(token, "ABORT") == 0) {
    abortTaskToIdle();
    return;
  }

  if (strcmp(token, "TASK") == 0) {
    char *taskAction = strtok(NULL, " ");
    if (taskAction == NULL) {
      setStatusNote("Bad TASK command");
      return;
    }

    if (strcmp(taskAction, "START") == 0) {
      MissionTask task = parseTaskId(strtok(NULL, " "));
      if (task == TASK_NONE) {
        setStatusNote("Bad TASK START id");
        return;
      }
      startTaskInternal(task);
      return;
    }

    if (strcmp(taskAction, "ABORT") == 0) {
      abortTaskToIdle();
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

  if (strcmp(token, "STOP") == 0) {
    applyManualDriveCommand(DRIVE_STOP, 0);
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

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  setupMotorPins();
  setupFrontUltrasonics();
  setupTaskStatusLeds();
  setupArm();
  setupCargoDoorServo();
  setupRfidReader();
  stopCar();

  if (!initMpu6050()) {
    mpuReady = false;
    enterFault("MPU6050 init fail");
    Serial.println(F("MPU6050 init failed"));
  } else {
    mpuReady = true;
    calibrateGyroBias();
    enterAutoStage(STAGE_IDLE);
  }

  delay(1000);
  printHelp();
}

void loop() {
  updateHeading();
  pollSerialCommands();
  updateDistanceSnapshot();
  updateRfidReader();
  runMissionStateMachine();
  updateDriveFailsafe();
  driveCurrentDirection();
  updateTaskStatusLeds();
  emitTelemetry();
}
