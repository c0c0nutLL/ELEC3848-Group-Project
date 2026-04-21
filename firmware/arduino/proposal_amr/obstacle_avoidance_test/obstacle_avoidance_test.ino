#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
  OBSOLETE / HISTORICAL REFERENCE

  This prototype depends on the removed side ultrasonic sensors to choose
  bypass direction and measure sidestep distance.
  It is kept only to preserve the old YOLO + ultrasonic experiment.
*/

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

// Ultrasonic pins reused from the existing reference sketches.
const uint8_t FRONT_RIGHT_TRIG_PIN = 6;
const uint8_t FRONT_RIGHT_ECHO_PIN = 4;
const uint8_t FRONT_LEFT_TRIG_PIN = 10;
const uint8_t FRONT_LEFT_ECHO_PIN = 7;
const uint8_t SIDE_LEFT_TRIG_PIN = 28;
const uint8_t SIDE_LEFT_ECHO_PIN = 25;
const uint8_t SIDE_RIGHT_TRIG_PIN = 30;
const uint8_t SIDE_RIGHT_ECHO_PIN = 32;
const uint8_t START_BUTTON_PIN = 33;

const uint8_t MPU6050_ADDR = 0x68;

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const int8_t OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long OLED_INTERVAL_MS = 120UL;
const unsigned long ULTRA_INTERVAL_MS = 90UL;
const unsigned long HEADING_INTERVAL_MS = 8UL;
const unsigned long ULTRA_TIMEOUT_US = 30000UL;
const unsigned long VISION_STALE_MS = 800UL;
const unsigned long CLEAR_STABLE_MS = 350UL;
const unsigned long PERSON_WAIT_MS = 2500UL;
const unsigned long OBJECT_WAIT_MS = 1500UL;
const unsigned long UNKNOWN_WAIT_MS = 1500UL;
const unsigned long SIDE_MOVE_TIMEOUT_MS = 1800UL;
const unsigned long PASS_PERSON_TIMEOUT_MS = 3200UL;
const unsigned long PASS_OBJECT_TIMEOUT_MS = 2600UL;
const unsigned long RETURN_TIMEOUT_MS = 1800UL;
const unsigned long RECOVER_TIMEOUT_MS = 2500UL;
const unsigned long RECOVER_STABLE_MS = 180UL;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float FRONT_BLOCK_CM = 50.0f;
const float FRONT_CLEAR_CM = 60.0f;
const float FRONT_CRITICAL_CM = 15.0f;
const float PERSON_SIDE_MIN_CM = 24.0f;
const float OBJECT_SIDE_MIN_CM = 20.0f;
const float UNKNOWN_SIDE_MIN_CM = 20.0f;
const float TARGET_SHIFT_CM = 16.0f;
const float RETURN_TOLERANCE_CM = 3.0f;
const float HEADING_TOLERANCE_DEG = 4.0f;
const float HEADING_FINE_TOLERANCE_DEG = 2.0f;

const int CRUISE_PWM = 34;
const int PERSON_PASS_PWM = 28;
const int OBJECT_PASS_PWM = 30;
const int STRAFE_PWM = 30;
const int RETURN_PWM = 30;
const int TURN_PWM_FAST = 28;
const int TURN_PWM_SLOW = 22;

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
  STATE_BLOCKED_WAIT,
  STATE_AVOID_DECIDE,
  STATE_AVOID_MOVE_SIDE,
  STATE_AVOID_PASS,
  STATE_AVOID_RETURN,
  STATE_RECOVER_HEADING,
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

bool testEnabled = false;
bool mpuReady = false;
bool lastButtonPressed = false;
bool faultNeedsManualReset = false;

unsigned long stateStartMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastUltraMs = 0;
unsigned long lastHeadingMs = 0;
unsigned long lastHeadingUs = 0;
unsigned long clearStableStartMs = 0;
unsigned long recoverStableStartMs = 0;
unsigned long invalidFrontSinceMs = 0;
unsigned long invalidSideSinceMs = 0;

float gyroZBias = 0.0f;
float latestGyroZDps = 0.0f;
float headingDeg = 0.0f;
float routeHeadingDeg = 0.0f;
DistancePair frontDistances = {-1.0f, -1.0f};
DistancePair sideDistances = {-1.0f, -1.0f};
float frontCm = -1.0f;
float sideBaselineLeftCm = -1.0f;
float sideBaselineRightCm = -1.0f;
float sideMoveTargetCm = -1.0f;
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
    case STATE_BLOCKED_WAIT: return "BLOCKED_WAIT";
    case STATE_AVOID_DECIDE: return "AVOID_DECIDE";
    case STATE_AVOID_MOVE_SIDE: return "AVOID_MOVE_SIDE";
    case STATE_AVOID_PASS: return "AVOID_PASS";
    case STATE_AVOID_RETURN: return "AVOID_RETURN";
    case STATE_RECOVER_HEADING: return "RECOVER_HEADING";
    case STATE_FAILSAFE: return "FAILSAFE";
    case STATE_STOPPED:
    default: return "STOPPED";
  }
}


const char *actionName() {
  switch (currentState) {
    case STATE_CLEAR: return "FORWARD";
    case STATE_BLOCKED_WAIT: return "WAIT";
    case STATE_AVOID_DECIDE: return "DECIDE";
    case STATE_AVOID_MOVE_SIDE: return chosenAvoidSide == AVOID_LEFT ? "AVOID_LEFT" : "AVOID_RIGHT";
    case STATE_AVOID_PASS: return "PASS";
    case STATE_AVOID_RETURN: return "RETURN";
    case STATE_RECOVER_HEADING: return "RECOVER";
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


void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}


int applyLeftTrim(int pwm) {
  return constrain(static_cast<int>(pwm * 1.10f), 0, 255);
}


void moveForward(int pwm) {
  int leftPwm = applyLeftTrim(pwm);
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(pwm);
}


void strafeLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}


void strafeRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
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


bool isButtonPressed() {
  return digitalRead(START_BUTTON_PIN) == LOW;
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


void setupUltrasonics() {
  pinMode(FRONT_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_ECHO_PIN, INPUT);
  pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ECHO_PIN, INPUT);
  pinMode(SIDE_LEFT_TRIG_PIN, OUTPUT);
  pinMode(SIDE_LEFT_ECHO_PIN, INPUT);
  pinMode(SIDE_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(SIDE_RIGHT_ECHO_PIN, INPUT);

  digitalWrite(FRONT_RIGHT_TRIG_PIN, LOW);
  digitalWrite(FRONT_LEFT_TRIG_PIN, LOW);
  digitalWrite(SIDE_LEFT_TRIG_PIN, LOW);
  digitalWrite(SIDE_RIGHT_TRIG_PIN, LOW);
}


void setupButton() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
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
  display.println("Obstacle test");
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
  const int sampleCount = 500;
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
  lastHeadingUs = 0;
}


void updateHeading() {
  if (!mpuReady) {
    return;
  }
  if (millis() - lastHeadingMs < HEADING_INTERVAL_MS) {
    return;
  }
  lastHeadingMs = millis();

  unsigned long nowUs = micros();
  if (lastHeadingUs == 0) {
    lastHeadingUs = nowUs;
    return;
  }

  float dt = (nowUs - lastHeadingUs) / 1000000.0f;
  lastHeadingUs = nowUs;

  float gyroZ = readGyroZDpsRaw() - gyroZBias;
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

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
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


void updateDistanceSnapshot() {
  if (millis() - lastUltraMs < ULTRA_INTERVAL_MS) {
    return;
  }

  lastUltraMs = millis();
  frontDistances.rightCm = readDistanceCm(FRONT_RIGHT_TRIG_PIN, FRONT_RIGHT_ECHO_PIN);
  delay(4);
  frontDistances.leftCm = readDistanceCm(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN);
  delay(4);
  sideDistances.leftCm = readDistanceCm(SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN);
  delay(4);
  sideDistances.rightCm = readDistanceCm(SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN);

  frontCm = averageValid(frontDistances.rightCm, frontDistances.leftCm);

  if (frontCm < 0.0f) {
    if (invalidFrontSinceMs == 0) {
      invalidFrontSinceMs = millis();
    }
  } else {
    invalidFrontSinceMs = 0;
  }

  if (sideDistances.leftCm < 0.0f || sideDistances.rightCm < 0.0f) {
    if (invalidSideSinceMs == 0) {
      invalidSideSinceMs = millis();
    }
  } else {
    invalidSideSinceMs = 0;
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
  return type == VISION_PERSON ? PASS_PERSON_TIMEOUT_MS : PASS_OBJECT_TIMEOUT_MS;
}


float sideMinClearanceFor(VisionType type) {
  if (type == VISION_PERSON) {
    return PERSON_SIDE_MIN_CM;
  }
  if (type == VISION_OBJECT) {
    return OBJECT_SIDE_MIN_CM;
  }
  return UNKNOWN_SIDE_MIN_CM;
}


int passPwmFor(VisionType type) {
  return type == VISION_PERSON ? PERSON_PASS_PWM : OBJECT_PASS_PWM;
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


void enterFailsafe(const char *message) {
  stopCar();
  setFault(message);
  faultNeedsManualReset = true;
  setState(STATE_FAILSAFE);
}


void beginTest() {
  if (!mpuReady) {
    setFault("MPU6050_NOT_READY");
    faultNeedsManualReset = true;
    setState(STATE_FAILSAFE);
    return;
  }

  clearFault();
  faultNeedsManualReset = false;
  testEnabled = true;
  latchedObstacleType = VISION_CLEAR;
  latchedObstacleZone = ZONE_NONE;
  chosenAvoidSide = AVOID_NONE;
  sideMoveTargetCm = -1.0f;
  sideBaselineLeftCm = sideDistances.leftCm;
  sideBaselineRightCm = sideDistances.rightCm;
  routeHeadingDeg = headingDeg;
  setState(STATE_CLEAR);
}


void stopTest() {
  testEnabled = false;
  latchedObstacleType = VISION_CLEAR;
  latchedObstacleZone = ZONE_NONE;
  chosenAvoidSide = AVOID_NONE;
  sideMoveTargetCm = -1.0f;
  stopCar();
  setState(STATE_STOPPED);
}


void maybeHandleSensorFaults() {
  if (!testEnabled || currentState == STATE_STOPPED || currentState == STATE_FAILSAFE) {
    return;
  }

  if (invalidFrontSinceMs > 0 && millis() - invalidFrontSinceMs > 700UL) {
    enterFailsafe("FRONT_ULTRA_TIMEOUT");
    return;
  }

  if ((currentState == STATE_AVOID_DECIDE ||
       currentState == STATE_AVOID_MOVE_SIDE ||
       currentState == STATE_AVOID_RETURN) &&
      invalidSideSinceMs > 0 &&
      millis() - invalidSideSinceMs > 700UL) {
    enterFailsafe("SIDE_ULTRA_TIMEOUT");
  }
}


void latchCurrentObstacle() {
  VisionType nowType = classifyBlockingObstacle();
  if (nowType == VISION_PERSON) {
    latchedObstacleType = VISION_PERSON;
  } else if (nowType == VISION_OBJECT && latchedObstacleType != VISION_PERSON) {
    latchedObstacleType = VISION_OBJECT;
  } else if (nowType == VISION_UNKNOWN && latchedObstacleType == VISION_CLEAR) {
    latchedObstacleType = VISION_UNKNOWN;
  }

  VisionZone nowZone = effectiveVisionZone();
  if (nowZone != ZONE_NONE) {
    latchedObstacleZone = nowZone;
  }
}


bool sideAvailable(AvoidSide side, float minClearanceCm) {
  float sideCm = side == AVOID_LEFT ? sideDistances.leftCm : sideDistances.rightCm;
  return sideCm >= minClearanceCm;
}


AvoidSide chooseAvoidSide() {
  // Prefer the safer opposite side for a detected person when possible.
  float minClearanceCm = sideMinClearanceFor(latchedObstacleType);
  bool leftOk = sideAvailable(AVOID_LEFT, minClearanceCm);
  bool rightOk = sideAvailable(AVOID_RIGHT, minClearanceCm);

  if (latchedObstacleType == VISION_PERSON) {
    if (latchedObstacleZone == ZONE_LEFT && rightOk) {
      return AVOID_RIGHT;
    }
    if (latchedObstacleZone == ZONE_RIGHT && leftOk) {
      return AVOID_LEFT;
    }
  }

  if (leftOk && rightOk) {
    if (fabs(sideDistances.leftCm - sideDistances.rightCm) < 2.0f) {
      if (latchedObstacleZone == ZONE_LEFT) {
        return AVOID_RIGHT;
      }
      if (latchedObstacleZone == ZONE_RIGHT) {
        return AVOID_LEFT;
      }
    }
    return sideDistances.leftCm > sideDistances.rightCm ? AVOID_LEFT : AVOID_RIGHT;
  }

  if (leftOk) {
    return AVOID_LEFT;
  }
  if (rightOk) {
    return AVOID_RIGHT;
  }
  return AVOID_NONE;
}


float chosenSideDistance(AvoidSide side) {
  if (side == AVOID_LEFT) {
    return sideDistances.leftCm;
  }
  if (side == AVOID_RIGHT) {
    return sideDistances.rightCm;
  }
  return -1.0f;
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
    sideBaselineLeftCm = sideDistances.leftCm;
    sideBaselineRightCm = sideDistances.rightCm;
    routeHeadingDeg = headingDeg;
    stopCar();
    setState(STATE_BLOCKED_WAIT);
    return;
  }

  driveForwardWithHeading(CRUISE_PWM, routeHeadingDeg);
}


void runBlockedWaitState() {
  // Stop first and wait before any active bypass motion.
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
      routeHeadingDeg = headingDeg;
      setState(STATE_CLEAR);
      return;
    }
  } else {
    clearStableStartMs = 0;
  }

  if (millis() - stateStartMs >= waitDurationFor(latchedObstacleType) && frontIsBlocked()) {
    setState(STATE_AVOID_DECIDE);
  }
}


void runAvoidDecideState() {
  if (!frontIsBlocked()) {
    setState(STATE_CLEAR);
    return;
  }

  chosenAvoidSide = chooseAvoidSide();
  if (chosenAvoidSide == AVOID_NONE) {
    enterFailsafe("NO_SAFE_SIDE");
    return;
  }

  float minClearanceCm = sideMinClearanceFor(latchedObstacleType);
  if (chosenAvoidSide == AVOID_LEFT) {
    sideMoveTargetCm = max(minClearanceCm, sideBaselineLeftCm - TARGET_SHIFT_CM);
  } else {
    sideMoveTargetCm = max(minClearanceCm, sideBaselineRightCm - TARGET_SHIFT_CM);
  }

  setState(STATE_AVOID_MOVE_SIDE);
}


void runAvoidMoveSideState() {
  // Side move is constrained only by local side range and gyro heading hold.
  if (millis() - stateStartMs >= SIDE_MOVE_TIMEOUT_MS) {
    enterFailsafe("SIDE_MOVE_TIMEOUT");
    return;
  }

  float currentSideCm = chosenSideDistance(chosenAvoidSide);
  if (currentSideCm < 0.0f) {
    enterFailsafe("SIDE_MOVE_NO_SENSOR");
    return;
  }

  if (currentSideCm <= sideMoveTargetCm) {
    stopCar();
    setState(STATE_AVOID_PASS);
    return;
  }

  strafeWithHeading(chosenAvoidSide, STRAFE_PWM, routeHeadingDeg);
}


void runAvoidPassState() {
  // The pass stage is a short local go-through, not global rerouting.
  if (frontIsCritical()) {
    enterFailsafe("PASS_FRONT_CRITICAL");
    return;
  }

  if (millis() - stateStartMs >= passTimeoutFor(latchedObstacleType)) {
    enterFailsafe("PASS_TIMEOUT");
    return;
  }

  if (frontIsClear()) {
    if (clearStableStartMs == 0) {
      clearStableStartMs = millis();
    }
    if (millis() - clearStableStartMs >= CLEAR_STABLE_MS) {
      stopCar();
      setState(STATE_AVOID_RETURN);
      return;
    }
  } else {
    clearStableStartMs = 0;
  }

  driveForwardWithHeading(passPwmFor(latchedObstacleType), routeHeadingDeg);
}


void runAvoidReturnState() {
  if (millis() - stateStartMs >= RETURN_TIMEOUT_MS) {
    enterFailsafe("RETURN_TIMEOUT");
    return;
  }

  AvoidSide returnSide = chosenAvoidSide == AVOID_LEFT ? AVOID_RIGHT : AVOID_LEFT;
  float targetCm = chosenAvoidSide == AVOID_LEFT ? sideBaselineLeftCm : sideBaselineRightCm;
  float currentSideCm = chosenSideDistance(chosenAvoidSide);
  if (currentSideCm < 0.0f) {
    enterFailsafe("RETURN_NO_SENSOR");
    return;
  }

  if (fabs(currentSideCm - targetCm) <= RETURN_TOLERANCE_CM) {
    stopCar();
    setState(STATE_RECOVER_HEADING);
    return;
  }

  strafeWithHeading(returnSide, RETURN_PWM, routeHeadingDeg);
}


void runRecoverHeadingState() {
  // Recovery only restores heading, not map-level route position.
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
      latchedObstacleType = VISION_CLEAR;
      latchedObstacleZone = ZONE_NONE;
      chosenAvoidSide = AVOID_NONE;
      routeHeadingDeg = headingDeg;
      setState(STATE_CLEAR);
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


void runFailsafeState() {
  stopCar();
}


void runStateMachine() {
  if (!testEnabled) {
    stopCar();
    return;
  }

  switch (currentState) {
    case STATE_CLEAR: runClearState(); break;
    case STATE_BLOCKED_WAIT: runBlockedWaitState(); break;
    case STATE_AVOID_DECIDE: runAvoidDecideState(); break;
    case STATE_AVOID_MOVE_SIDE: runAvoidMoveSideState(); break;
    case STATE_AVOID_PASS: runAvoidPassState(); break;
    case STATE_AVOID_RETURN: runAvoidReturnState(); break;
    case STATE_RECOVER_HEADING: runRecoverHeadingState(); break;
    case STATE_FAILSAFE: runFailsafeState(); break;
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
  // Keep the serial protocol simple and line-based.
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
  VisionType currentObstacle = classifyBlockingObstacle();

  Serial.print('{');
  Serial.print("\"ts_ms\":"); Serial.print(millis()); Serial.print(',');
  Serial.print("\"state\":"); printJsonString(stateName(currentState)); Serial.print(',');
  Serial.print("\"action\":"); printJsonString(actionName()); Serial.print(',');
  Serial.print("\"test_enabled\":"); Serial.print(testEnabled ? "true" : "false"); Serial.print(',');
  Serial.print("\"front_cm\":"); printJsonFloat(frontCm, 1); Serial.print(',');
  Serial.print("\"front_right_cm\":"); printJsonFloat(frontDistances.rightCm, 1); Serial.print(',');
  Serial.print("\"front_left_cm\":"); printJsonFloat(frontDistances.leftCm, 1); Serial.print(',');
  Serial.print("\"side_left_cm\":"); printJsonFloat(sideDistances.leftCm, 1); Serial.print(',');
  Serial.print("\"side_right_cm\":"); printJsonFloat(sideDistances.rightCm, 1); Serial.print(',');
  Serial.print("\"heading_deg\":"); Serial.print(headingDeg, 1); Serial.print(',');
  Serial.print("\"gyro_z_dps\":"); Serial.print(latestGyroZDps, 2); Serial.print(',');
  Serial.print("\"vision_type\":"); printJsonString(visionTypeName(effectiveVisionType())); Serial.print(',');
  Serial.print("\"vision_zone\":"); printJsonString(visionZoneName(effectiveVisionZone())); Serial.print(',');
  Serial.print("\"vision_confidence\":"); Serial.print(latestVision.confidence); Serial.print(',');
  Serial.print("\"vision_age_ms\":");
  if (latestVision.lastUpdateMs == 0) {
    Serial.print("null");
  } else {
    Serial.print(millis() - latestVision.lastUpdateMs);
  }
  Serial.print(',');
  Serial.print("\"obstacle_type\":"); printJsonString(visionTypeName(currentObstacle)); Serial.print(',');
  Serial.print("\"latched_obstacle_type\":"); printJsonString(visionTypeName(latchedObstacleType)); Serial.print(',');
  Serial.print("\"latched_obstacle_zone\":"); printJsonString(visionZoneName(latchedObstacleZone)); Serial.print(',');
  Serial.print("\"avoid_side\":"); printJsonString(avoidSideName(chosenAvoidSide)); Serial.print(',');
  Serial.print("\"fault\":"); printJsonString(faultMessage);
  Serial.println('}');
}


void printFloatOrDash(float value) {
  if (value < 0.0f) {
    display.print("--");
  } else {
    display.print(value, 0);
  }
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
  printFloatOrDash(frontCm);
  display.print(" L:");
  printFloatOrDash(sideDistances.leftCm);
  display.print(" R:");
  printFloatOrDash(sideDistances.rightCm);
  display.println();

  display.print("OBS:");
  display.print(visionTypeName(classifyBlockingObstacle()));
  display.print(" ");
  display.println(visionZoneName(effectiveVisionZone()));

  display.print("ACT:");
  display.print(actionName());
  display.print(" ");
  display.println(avoidSideName(chosenAvoidSide));
  display.display();
}


void printHelp() {
  Serial.println("Obstacle avoidance test ready.");
  Serial.println("OBSOLETE: side ultrasonic hardware was removed on 2026-04-21.");
  Serial.println("Do not use this sketch as a current hardware target.");
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
  setupUltrasonics();
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
  updateDistanceSnapshot();
  maybeHandleSensorFaults();
  runStateMachine();
  updateOled();
  emitTelemetry();
}
