#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor pins reused from the current project baseline.
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

// Front ultrasonic pair remains active in the current hardware baseline.
const uint8_t FRONT_RIGHT_TRIG_PIN = 6;
const uint8_t FRONT_RIGHT_ECHO_PIN = 4;
const uint8_t FRONT_LEFT_TRIG_PIN = 10;
const uint8_t FRONT_LEFT_ECHO_PIN = 7;
const uint8_t START_BUTTON_PIN = 33;
const uint8_t MPU6050_ADDR = 0x68;

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const int8_t OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long OLED_INTERVAL_MS = 120UL;
const unsigned long FRONT_SAMPLE_INTERVAL_MS = 90UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long VISION_STALE_MS = 800UL;
const unsigned long CLEAR_STABLE_MS = 350UL;
const unsigned long PERSON_WAIT_MS = 2500UL;
const unsigned long OBJECT_WAIT_MS = 1500UL;
const unsigned long UNKNOWN_WAIT_MS = 1500UL;
const unsigned long STRAFE_MOVE_MS = 850UL;
const unsigned long RETURN_MOVE_MS = 850UL;
const unsigned long PASS_MIN_MS = 1100UL;
const unsigned long PASS_PERSON_MAX_MS = 3200UL;
const unsigned long PASS_OBJECT_MAX_MS = 2600UL;
const unsigned long RECOVER_TIMEOUT_MS = 2200UL;
const unsigned long DONE_HOLD_MS = 180UL;
const unsigned long SENSOR_FAULT_MS = 700UL;
const unsigned long TOTAL_AVOID_TIMEOUT_MS = 7000UL;
const unsigned long RECOVER_STABLE_MS = 180UL;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float FRONT_BLOCK_CM = 50.0f;
const float FRONT_CLEAR_CM = 65.0f;
const float FRONT_CRITICAL_CM = 14.0f;
const float HEADING_TOLERANCE_DEG = 4.0f;
const float HEADING_FINE_TOLERANCE_DEG = 2.0f;

const int CRUISE_PWM = 34;
const int PASS_PERSON_PWM = 30;
const int PASS_OBJECT_PWM = 32;
const int STRAFE_PWM = 34;
const int TURN_PWM_FAST = 30;
const int TURN_PWM_SLOW = 24;

const float FORWARD_LEFT_TRIM = 1.10f;
const float STRAFE_LEFT_RF_TRIM = 1.05f;
const float STRAFE_RIGHT_RF_TRIM = 1.10f;
const float STRAFE_RIGHT_LR_TRIM = 1.10f;

enum VisionType {
  VISION_CLEAR,
  VISION_PERSON,
  VISION_OBJECT,
  VISION_UNKNOWN
};

enum VisionZone {
  ZONE_NONE,
  ZONE_LEFT,
  ZONE_CENTER,
  ZONE_RIGHT
};

enum AvoidSide {
  AVOID_NONE,
  AVOID_LEFT,
  AVOID_RIGHT
};

enum TestState {
  STATE_STOPPED,
  STATE_CLEAR,
  STATE_WAIT_OBSTACLE,
  STATE_DECIDE_SIDE,
  STATE_STRAFE_LEFT,
  STATE_STRAFE_RIGHT,
  STATE_PASS_FORWARD,
  STATE_RETURN_TO_PATH,
  STATE_RECOVER_HEADING,
  STATE_DONE,
  STATE_FAILSAFE
};

struct DistancePair {
  float rightCm;
  float leftCm;
};

struct VisionObservation {
  VisionType type;
  VisionZone zone;
  int confidence;
  unsigned long lastUpdateMs;
};

char commandBuffer[64];
uint8_t commandLength = 0;

TestState currentState = STATE_STOPPED;
VisionObservation latestVision = {VISION_CLEAR, ZONE_NONE, 0, 0UL};
VisionType latchedObstacleType = VISION_CLEAR;
VisionZone latchedObstacleZone = ZONE_NONE;
AvoidSide chosenAvoidSide = AVOID_NONE;

unsigned long stateStartMs = 0;
unsigned long obstacleSequenceStartMs = 0;
unsigned long clearStableStartMs = 0;
unsigned long recoverStableStartMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastFrontSampleMs = 0;
unsigned long lastHeadingSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;
unsigned long invalidFrontSinceMs = 0;

bool lastButtonPressed = false;
bool mpuReady = false;
bool faultNeedsManualReset = false;
uint8_t gyroReadFailCount = 0;

float gyroZBias = 0.0f;
float latestGyroZDps = 0.0f;
float headingDeg = 0.0f;
float routeHeadingDeg = 0.0f;
DistancePair frontDistances = {-1.0f, -1.0f};
float frontCm = -1.0f;
char faultMessage[48] = "";


const char *visionTypeName(VisionType type) {
  switch (type) {
    case VISION_PERSON: return "PERSON";
    case VISION_OBJECT: return "OBJECT";
    case VISION_UNKNOWN: return "UNKNOWN";
    case VISION_CLEAR:
    default: return "CLEAR";
  }
}


const char *visionZoneName(VisionZone zone) {
  switch (zone) {
    case ZONE_LEFT: return "LEFT";
    case ZONE_CENTER: return "CENTER";
    case ZONE_RIGHT: return "RIGHT";
    case ZONE_NONE:
    default: return "NONE";
  }
}


const char *avoidSideName(AvoidSide side) {
  switch (side) {
    case AVOID_LEFT: return "LEFT";
    case AVOID_RIGHT: return "RIGHT";
    case AVOID_NONE:
    default: return "NONE";
  }
}


const char *stateName(TestState state) {
  switch (state) {
    case STATE_CLEAR: return "CLEAR";
    case STATE_WAIT_OBSTACLE: return "WAIT_OBSTACLE";
    case STATE_DECIDE_SIDE: return "DECIDE_SIDE";
    case STATE_STRAFE_LEFT: return "STRAFE_LEFT";
    case STATE_STRAFE_RIGHT: return "STRAFE_RIGHT";
    case STATE_PASS_FORWARD: return "PASS_FORWARD";
    case STATE_RETURN_TO_PATH: return "RETURN_TO_PATH";
    case STATE_RECOVER_HEADING: return "RECOVER_HEADING";
    case STATE_DONE: return "DONE";
    case STATE_FAILSAFE: return "FAILSAFE";
    case STATE_STOPPED:
    default: return "STOPPED";
  }
}


const char *actionName() {
  switch (currentState) {
    case STATE_CLEAR: return "FORWARD";
    case STATE_WAIT_OBSTACLE: return "WAIT";
    case STATE_DECIDE_SIDE: return "DECIDE";
    case STATE_STRAFE_LEFT: return "STRAFE_L";
    case STATE_STRAFE_RIGHT: return "STRAFE_R";
    case STATE_PASS_FORWARD: return "PASS";
    case STATE_RETURN_TO_PATH: return "RETURN";
    case STATE_RECOVER_HEADING: return "RECOVER";
    case STATE_DONE: return "DONE";
    case STATE_FAILSAFE: return "FAIL";
    case STATE_STOPPED:
    default: return "STOP";
  }
}


void setFault(const char *message) {
  strncpy(faultMessage, message, sizeof(faultMessage) - 1);
  faultMessage[sizeof(faultMessage) - 1] = '\0';
}


void clearFault() {
  faultMessage[0] = '\0';
}


void setState(TestState nextState) {
  currentState = nextState;
  stateStartMs = millis();
  clearStableStartMs = 0;
  recoverStableStartMs = 0;
}


int trimPwm(int pwm, float factor) {
  return constrain(static_cast<int>(pwm * factor), 0, 255);
}


void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}


void moveForward(int pwm) {
  int leftPwm = trimPwm(pwm, FORWARD_LEFT_TRIM);
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(pwm);
}


void strafeLeft(int pwm) {
  int rightFrontPwm = trimPwm(pwm, STRAFE_LEFT_RF_TRIM);
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(rightFrontPwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}


void strafeRight(int pwm) {
  int rightFrontPwm = trimPwm(pwm, STRAFE_RIGHT_RF_TRIM);
  int leftRearPwm = trimPwm(pwm, STRAFE_RIGHT_LR_TRIM);
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(rightFrontPwm);
  MOTORC_BACKOFF(leftRearPwm);
  MOTORD_BACKOFF(pwm);
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
  pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
  pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_RIGHT_TRIG_PIN, LOW);
  digitalWrite(FRONT_LEFT_TRIG_PIN, LOW);
}


void setupButton() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
}


bool isButtonPressed() {
  return digitalRead(START_BUTTON_PIN) == LOW;
}


void setupOled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    return;
  }
  oledReady = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Strafe test");
  display.display();
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


float readGyroZDpsRaw() {
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
  delay(400);

  for (int i = 0; i < sampleCount; ++i) {
    sum += readGyroZDpsRaw();
    delay(4);
  }

  gyroZBias = sum / sampleCount;
  latestGyroZDps = 0.0f;
  headingDeg = 0.0f;
  routeHeadingDeg = 0.0f;
  lastHeadingUpdateUs = 0;
  lastHeadingSampleMs = 0;
  gyroReadFailCount = 0;
}


void updateHeading() {
  if (!mpuReady) {
    return;
  }

  if (millis() - lastHeadingSampleMs < HEADING_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastHeadingSampleMs = millis();

  unsigned long nowUs = micros();
  if (lastHeadingUpdateUs == 0) {
    lastHeadingUpdateUs = nowUs;
    return;
  }

  float dt = (nowUs - lastHeadingUpdateUs) / 1000000.0f;
  lastHeadingUpdateUs = nowUs;

  uint8_t buffer[2];
  if (!readMpuBytes(0x47, 2, buffer)) {
    if (gyroReadFailCount < 255) {
      gyroReadFailCount++;
    }
    latestGyroZDps = 0.0f;
    return;
  }

  gyroReadFailCount = 0;
  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  float gyroZ = static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB - gyroZBias;
  latestGyroZDps = gyroZ;
  headingDeg += gyroZ * dt;
}


float angleErrorDeg(float targetDeg, float currentDeg) {
  float error = targetDeg - currentDeg;
  while (error > 180.0f) {
    error -= 360.0f;
  }
  while (error < -180.0f) {
    error += 360.0f;
  }
  return error;
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


float averageValid(float a, float b) {
  bool validA = a >= 0.0f;
  bool validB = b >= 0.0f;
  if (validA && validB) {
    return (a + b) * 0.5f;
  }
  if (validA) {
    return a;
  }
  if (validB) {
    return b;
  }
  return -1.0f;
}


void updateFrontSnapshot() {
  if (millis() - lastFrontSampleMs < FRONT_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastFrontSampleMs = millis();
  frontDistances.rightCm = readDistanceCm(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
  delay(4);
  frontDistances.leftCm = readDistanceCm(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
  frontCm = averageValid(frontDistances.rightCm, frontDistances.leftCm);

  if (frontCm < 0.0f) {
    if (invalidFrontSinceMs == 0) {
      invalidFrontSinceMs = millis();
    }
  } else {
    invalidFrontSinceMs = 0;
  }
}


bool frontIsBlocked() {
  return frontCm >= 0.0f && frontCm <= FRONT_BLOCK_CM;
}


bool frontIsClear() {
  return frontCm >= FRONT_CLEAR_CM;
}


bool frontIsCritical() {
  return frontCm >= 0.0f && frontCm <= FRONT_CRITICAL_CM;
}


VisionType effectiveVisionType() {
  if (millis() - latestVision.lastUpdateMs > VISION_STALE_MS) {
    return VISION_CLEAR;
  }
  return latestVision.type;
}


VisionZone effectiveVisionZone() {
  if (millis() - latestVision.lastUpdateMs > VISION_STALE_MS) {
    return ZONE_NONE;
  }
  return latestVision.zone;
}


VisionType classifyBlockingObstacle() {
  if (!frontIsBlocked()) {
    return VISION_CLEAR;
  }
  if (effectiveVisionType() == VISION_PERSON) {
    return VISION_PERSON;
  }
  if (effectiveVisionType() == VISION_OBJECT) {
    return VISION_OBJECT;
  }
  return VISION_UNKNOWN;
}


unsigned long waitDurationFor(VisionType type) {
  if (type == VISION_PERSON) {
    return PERSON_WAIT_MS;
  }
  if (type == VISION_OBJECT) {
    return OBJECT_WAIT_MS;
  }
  return UNKNOWN_WAIT_MS;
}


unsigned long passTimeoutFor(VisionType type) {
  return type == VISION_PERSON ? PASS_PERSON_MAX_MS : PASS_OBJECT_MAX_MS;
}


int passPwmFor(VisionType type) {
  return type == VISION_PERSON ? PASS_PERSON_PWM : PASS_OBJECT_PWM;
}


AvoidSide chooseAvoidSide() {
  if (latchedObstacleZone == ZONE_LEFT) {
    return AVOID_RIGHT;
  }
  if (latchedObstacleZone == ZONE_RIGHT) {
    return AVOID_LEFT;
  }
  return AVOID_RIGHT;
}


void driveForwardWithHeading(int pwm, float targetHeadingDeg) {
  float error = angleErrorDeg(targetHeadingDeg, headingDeg);
  if (fabs(error) <= HEADING_TOLERANCE_DEG) {
    moveForward(pwm);
    return;
  }

  if (error > 0.0f) {
    rotateLeft(fabs(error) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
  } else {
    rotateRight(fabs(error) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
  }
}


void strafeWithHeading(AvoidSide side, int pwm, float targetHeadingDeg) {
  float error = angleErrorDeg(targetHeadingDeg, headingDeg);
  if (fabs(error) > HEADING_TOLERANCE_DEG) {
    if (error > 0.0f) {
      rotateLeft(fabs(error) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
    } else {
      rotateRight(fabs(error) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
    }
    return;
  }

  if (side == AVOID_LEFT) {
    strafeLeft(pwm);
  } else if (side == AVOID_RIGHT) {
    strafeRight(pwm);
  } else {
    stopCar();
  }
}


void latchCurrentObstacle() {
  VisionType currentType = classifyBlockingObstacle();
  if (currentType == VISION_PERSON) {
    latchedObstacleType = VISION_PERSON;
  } else if (currentType == VISION_OBJECT && latchedObstacleType != VISION_PERSON) {
    latchedObstacleType = VISION_OBJECT;
  } else if (currentType == VISION_UNKNOWN && latchedObstacleType == VISION_CLEAR) {
    latchedObstacleType = VISION_UNKNOWN;
  }

  VisionZone currentZone = effectiveVisionZone();
  if (currentZone != ZONE_NONE) {
    latchedObstacleZone = currentZone;
  }
}


void enterFailsafe(const char *message) {
  stopCar();
  setFault(message);
  faultNeedsManualReset = true;
  setState(STATE_FAILSAFE);
}


void beginTest() {
  if (faultNeedsManualReset) {
    setState(STATE_FAILSAFE);
    return;
  }

  if (!mpuReady) {
    setFault("MPU6050_NOT_READY");
    faultNeedsManualReset = true;
    setState(STATE_FAILSAFE);
    return;
  }

  clearFault();
  faultNeedsManualReset = false;
  latchedObstacleType = VISION_CLEAR;
  latchedObstacleZone = ZONE_NONE;
  chosenAvoidSide = AVOID_NONE;
  routeHeadingDeg = headingDeg;
  obstacleSequenceStartMs = 0;
  setState(STATE_CLEAR);
}


void stopTest() {
  stopCar();
  latchedObstacleType = VISION_CLEAR;
  latchedObstacleZone = ZONE_NONE;
  chosenAvoidSide = AVOID_NONE;
  obstacleSequenceStartMs = 0;
  setState(STATE_STOPPED);
}


void maybeHandleSensorFaults() {
  if (currentState == STATE_STOPPED || currentState == STATE_FAILSAFE) {
    return;
  }

  if (invalidFrontSinceMs > 0 && millis() - invalidFrontSinceMs > SENSOR_FAULT_MS) {
    enterFailsafe("FRONT_ULTRA_TIMEOUT");
    return;
  }

  if (gyroReadFailCount >= 5) {
    enterFailsafe("MPU6050_READ_FAIL");
    return;
  }

  if (obstacleSequenceStartMs > 0 && millis() - obstacleSequenceStartMs > TOTAL_AVOID_TIMEOUT_MS) {
    enterFailsafe("AVOID_TOTAL_TIMEOUT");
  }
}


void runClearState() {
  if (frontIsCritical()) {
    enterFailsafe("FRONT_CRITICAL_STOP");
    return;
  }

  if (frontIsBlocked()) {
    latchedObstacleType = classifyBlockingObstacle();
    latchedObstacleZone = effectiveVisionZone();
    chosenAvoidSide = AVOID_NONE;
    obstacleSequenceStartMs = millis();
    routeHeadingDeg = headingDeg;
    stopCar();
    setState(STATE_WAIT_OBSTACLE);
    return;
  }

  driveForwardWithHeading(CRUISE_PWM, routeHeadingDeg);
}


void runWaitObstacleState() {
  stopCar();
  latchCurrentObstacle();

  if (!frontIsBlocked()) {
    if (clearStableStartMs == 0) {
      clearStableStartMs = millis();
    }
    if (millis() - clearStableStartMs >= CLEAR_STABLE_MS) {
      latchedObstacleType = VISION_CLEAR;
      latchedObstacleZone = ZONE_NONE;
      chosenAvoidSide = AVOID_NONE;
      obstacleSequenceStartMs = 0;
      routeHeadingDeg = headingDeg;
      setState(STATE_CLEAR);
      return;
    }
  } else {
    clearStableStartMs = 0;
  }

  if (millis() - stateStartMs >= waitDurationFor(latchedObstacleType) && frontIsBlocked()) {
    setState(STATE_DECIDE_SIDE);
  }
}


void runDecideSideState() {
  if (!frontIsBlocked()) {
    setState(STATE_CLEAR);
    return;
  }

  chosenAvoidSide = chooseAvoidSide();
  if (chosenAvoidSide == AVOID_LEFT) {
    setState(STATE_STRAFE_LEFT);
  } else {
    setState(STATE_STRAFE_RIGHT);
  }
}


void runStrafeState(AvoidSide side) {
  if (frontIsCritical()) {
    enterFailsafe("STRAFE_FRONT_CRITICAL");
    return;
  }

  if (millis() - stateStartMs >= STRAFE_MOVE_MS) {
    stopCar();
    setState(STATE_PASS_FORWARD);
    return;
  }

  strafeWithHeading(side, STRAFE_PWM, routeHeadingDeg);
}


void runPassForwardState() {
  if (frontIsCritical()) {
    enterFailsafe("PASS_FRONT_CRITICAL");
    return;
  }

  if (millis() - stateStartMs >= passTimeoutFor(latchedObstacleType)) {
    enterFailsafe("PASS_TIMEOUT");
    return;
  }

  if (millis() - stateStartMs >= PASS_MIN_MS && frontIsClear()) {
    if (clearStableStartMs == 0) {
      clearStableStartMs = millis();
    }
    if (millis() - clearStableStartMs >= CLEAR_STABLE_MS) {
      stopCar();
      setState(STATE_RETURN_TO_PATH);
      return;
    }
  } else {
    clearStableStartMs = 0;
  }

  driveForwardWithHeading(passPwmFor(latchedObstacleType), routeHeadingDeg);
}


void runReturnState() {
  AvoidSide returnSide = chosenAvoidSide == AVOID_LEFT ? AVOID_RIGHT : AVOID_LEFT;
  if (millis() - stateStartMs >= RETURN_MOVE_MS) {
    stopCar();
    setState(STATE_RECOVER_HEADING);
    return;
  }

  strafeWithHeading(returnSide, STRAFE_PWM, routeHeadingDeg);
}


void runRecoverHeadingState() {
  if (millis() - stateStartMs >= RECOVER_TIMEOUT_MS) {
    enterFailsafe("RECOVER_TIMEOUT");
    return;
  }

  float errorDeg = angleErrorDeg(routeHeadingDeg, headingDeg);
  if (fabs(errorDeg) <= HEADING_FINE_TOLERANCE_DEG) {
    stopCar();
    if (recoverStableStartMs == 0) {
      recoverStableStartMs = millis();
    }
    if (millis() - recoverStableStartMs >= RECOVER_STABLE_MS) {
      setState(STATE_DONE);
    }
    return;
  }

  recoverStableStartMs = 0;
  if (errorDeg > 0.0f) {
    rotateLeft(fabs(errorDeg) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
  } else {
    rotateRight(fabs(errorDeg) > 8.0f ? TURN_PWM_FAST : TURN_PWM_SLOW);
  }
}


void runDoneState() {
  stopCar();
  if (millis() - stateStartMs >= DONE_HOLD_MS) {
    latchedObstacleType = VISION_CLEAR;
    latchedObstacleZone = ZONE_NONE;
    chosenAvoidSide = AVOID_NONE;
    obstacleSequenceStartMs = 0;
    routeHeadingDeg = headingDeg;
    setState(STATE_CLEAR);
  }
}


void runFailsafeState() {
  stopCar();
}


void runStateMachine() {
  switch (currentState) {
    case STATE_CLEAR:
      runClearState();
      break;
    case STATE_WAIT_OBSTACLE:
      runWaitObstacleState();
      break;
    case STATE_DECIDE_SIDE:
      runDecideSideState();
      break;
    case STATE_STRAFE_LEFT:
      runStrafeState(AVOID_LEFT);
      break;
    case STATE_STRAFE_RIGHT:
      runStrafeState(AVOID_RIGHT);
      break;
    case STATE_PASS_FORWARD:
      runPassForwardState();
      break;
    case STATE_RETURN_TO_PATH:
      runReturnState();
      break;
    case STATE_RECOVER_HEADING:
      runRecoverHeadingState();
      break;
    case STATE_DONE:
      runDoneState();
      break;
    case STATE_FAILSAFE:
      runFailsafeState();
      break;
    case STATE_STOPPED:
    default:
      stopCar();
      break;
  }
}


VisionType parseVisionTypeToken(const char *token) {
  if (strcmp(token, "PERSON") == 0) {
    return VISION_PERSON;
  }
  if (strcmp(token, "OBJECT") == 0) {
    return VISION_OBJECT;
  }
  if (strcmp(token, "UNKNOWN") == 0) {
    return VISION_UNKNOWN;
  }
  return VISION_CLEAR;
}


VisionZone parseVisionZoneToken(const char *token) {
  if (strcmp(token, "LEFT") == 0) {
    return ZONE_LEFT;
  }
  if (strcmp(token, "CENTER") == 0) {
    return ZONE_CENTER;
  }
  if (strcmp(token, "RIGHT") == 0) {
    return ZONE_RIGHT;
  }
  return ZONE_NONE;
}


void updateVisionObservation(VisionType type, VisionZone zone, int confidence) {
  latestVision.type = type;
  latestVision.zone = zone;
  latestVision.confidence = confidence;
  latestVision.lastUpdateMs = millis();
}


void handleCommand(const char *command) {
  char buffer[64];
  strncpy(buffer, command, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "START") == 0) {
    if (!faultNeedsManualReset) {
      beginTest();
    }
    return;
  }

  if (strcmp(token, "STOP") == 0) {
    stopTest();
    return;
  }

  if (strcmp(token, "RESET") == 0) {
    clearFault();
    faultNeedsManualReset = false;
    stopTest();
    return;
  }

  if (strcmp(token, "OBS") == 0) {
    char *typeToken = strtok(NULL, " ");
    char *zoneToken = strtok(NULL, " ");
    char *confidenceToken = strtok(NULL, " ");
    if (typeToken == NULL || zoneToken == NULL) {
      return;
    }

    int confidence = 0;
    if (confidenceToken != NULL) {
      confidence = atoi(confidenceToken);
    }
    updateVisionObservation(
      parseVisionTypeToken(typeToken),
      parseVisionZoneToken(zoneToken),
      confidence
    );
  }
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
  float headingError = angleErrorDeg(routeHeadingDeg, headingDeg);

  Serial.print('{');
  Serial.print("\"ts_ms\":"); Serial.print(millis()); Serial.print(',');
  Serial.print("\"state\":"); printJsonString(stateName(currentState)); Serial.print(',');
  Serial.print("\"action\":"); printJsonString(actionName()); Serial.print(',');
  Serial.print("\"front_cm\":"); printJsonFloat(frontCm, 1); Serial.print(',');
  Serial.print("\"front_right_cm\":"); printJsonFloat(frontDistances.rightCm, 1); Serial.print(',');
  Serial.print("\"front_left_cm\":"); printJsonFloat(frontDistances.leftCm, 1); Serial.print(',');
  Serial.print("\"heading_deg\":"); Serial.print(headingDeg, 2); Serial.print(',');
  Serial.print("\"heading_target_deg\":"); Serial.print(routeHeadingDeg, 2); Serial.print(',');
  Serial.print("\"heading_error_deg\":"); Serial.print(headingError, 2); Serial.print(',');
  Serial.print("\"gyro_z_dps\":"); Serial.print(latestGyroZDps, 2); Serial.print(',');
  Serial.print("\"vision_type\":"); printJsonString(visionTypeName(effectiveVisionType())); Serial.print(',');
  Serial.print("\"vision_zone\":"); printJsonString(visionZoneName(effectiveVisionZone())); Serial.print(',');
  Serial.print("\"vision_confidence\":"); Serial.print(latestVision.confidence); Serial.print(',');
  Serial.print("\"obstacle_type\":"); printJsonString(visionTypeName(classifyBlockingObstacle())); Serial.print(',');
  Serial.print("\"latched_obstacle_type\":"); printJsonString(visionTypeName(latchedObstacleType)); Serial.print(',');
  Serial.print("\"latched_obstacle_zone\":"); printJsonString(visionZoneName(latchedObstacleZone)); Serial.print(',');
  Serial.print("\"avoid_side\":"); printJsonString(avoidSideName(chosenAvoidSide)); Serial.print(',');
  Serial.print("\"fault\":"); printJsonString(faultMessage);
  Serial.println('}');
}


void updateOled() {
  if (!oledReady || millis() - lastOledMs < OLED_INTERVAL_MS) {
    return;
  }

  lastOledMs = millis();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("ST:");
  display.println(stateName(currentState));

  display.print("F:");
  if (frontCm < 0.0f) {
    display.print("--");
  } else {
    display.print(frontCm, 0);
  }
  display.print(" O:");
  display.println(visionTypeName(latchedObstacleType == VISION_CLEAR ? classifyBlockingObstacle() : latchedObstacleType));

  display.print("A:");
  display.println(actionName());

  display.print("HE:");
  display.print(angleErrorDeg(routeHeadingDeg, headingDeg), 1);
  display.display();
}


void printHelp() {
  Serial.println("Strafe obstacle test ready.");
  Serial.println("Commands:");
  Serial.println("  START");
  Serial.println("  STOP");
  Serial.println("  RESET");
  Serial.println("  OBS <CLEAR|PERSON|OBJECT|UNKNOWN> <NONE|LEFT|CENTER|RIGHT> <0-100>");
}


void handleButton() {
  bool buttonPressed = isButtonPressed();
  if (buttonPressed && !lastButtonPressed) {
    if (currentState == STATE_STOPPED) {
      beginTest();
    } else if (currentState == STATE_FAILSAFE) {
      clearFault();
      faultNeedsManualReset = false;
      stopTest();
    }
  }
  lastButtonPressed = buttonPressed;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  setupMotorPins();
  setupFrontUltrasonics();
  setupButton();
  setupOled();
  stopCar();

  if (!initMpu6050()) {
    setFault("MPU6050_INIT_FAIL");
    faultNeedsManualReset = true;
    mpuReady = false;
    setState(STATE_FAILSAFE);
  } else {
    mpuReady = true;
    calibrateGyroBias();
    setState(STATE_STOPPED);
  }

  printHelp();
}


void loop() {
  pollSerialCommands();
  handleButton();
  updateHeading();
  updateFrontSnapshot();
  maybeHandleSensorFaults();
  runStateMachine();
  updateOled();
  emitTelemetry();
}
