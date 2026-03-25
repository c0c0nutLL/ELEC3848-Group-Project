#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor pin mapping follows reference.ino and the existing test sketches.
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

// Pin mapping verified against hardware/amr_pin_mapping_template.xlsx.
const uint8_t ULTRASONIC_RIGHT_TRIG_PIN = 6;
const uint8_t ULTRASONIC_RIGHT_ECHO_PIN = 4;
const uint8_t ULTRASONIC_LEFT_TRIG_PIN = 10;
const uint8_t ULTRASONIC_LEFT_ECHO_PIN = 7;
const uint8_t START_BUTTON_PIN = 33;

const uint8_t COLOR_OUT_PIN = 46;
const uint8_t COLOR_S0_PIN = 44;
const uint8_t COLOR_S1_PIN = 45;
const uint8_t COLOR_S2_PIN = 13;
const uint8_t COLOR_S3_PIN = 11;
const uint8_t COLOR_LED_PIN = 47;

const uint8_t MPU6050_ADDR = 0x68;

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const int8_t OLED_RESET = 28;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long OLED_INTERVAL_MS = 120UL;
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 80UL;
const unsigned long COLOR_SAMPLE_INTERVAL_MS = 70UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long SETTLE_TIME_MS = 180UL;
const unsigned long ALIGN_STABLE_MS = 160UL;
const unsigned long TURN_STABLE_MS = 180UL;
const unsigned long COLOR_CONFIRM_TIME_MS = 2000UL;
const unsigned long COLOR_SEARCH_LEFT_MS = 4000UL;
const unsigned long COLOR_SEARCH_RIGHT_MS = 7000UL;
const unsigned long TURN_TIMEOUT_MS = 6000UL;

const float FIRST_TARGET_CM = 27.0f;
const float PRETURN_DISTANCE_TOLERANCE_CM = 0.8f;
const float FIRST_RECOVER_LOW_CM = 25.5f;
const float FIRST_RECOVER_HIGH_CM = 28.5f;
const float FIRST_STAGE_TRANSITION_CM = 65.0f;
const float FIRST_SLOW_START_CM = 42.0f;
const float FIRST_CRAWL_START_CM = 31.0f;

const float SECOND_TARGET_CM = 9.0f;
const float SECOND_RECOVER_LOW_CM = 8.0f;
const float SECOND_RECOVER_HIGH_CM = 10.0f;
const float SECOND_SLOW_START_CM = 20.0f;
const float SECOND_CRAWL_START_CM = 12.0f;
const float FINAL_HARD_STOP_CM = 6.0f;

const float ALIGN_TOLERANCE_CM = 1.4f;
const float FINAL_ALIGN_TOLERANCE_CM = 1.0f;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float TURN_ANGLE_DEG = 90.0f;
const float TURN_TOLERANCE_DEG = 2.2f;
const float HEADING_HOLD_TOLERANCE_DEG = 3.0f;

const int FORWARD_PWM_FAST = 36;
const int FORWARD_PWM_SLOW = 31;
const int FORWARD_PWM_CRAWL = 27;
const int FINAL_FORWARD_PWM_FAST = 32;
const int FINAL_FORWARD_PWM_SLOW = 27;
const int FINAL_FORWARD_PWM_CRAWL = 23;
const int STRAFE_PWM = 29;
const int RIGHT_STRAFE_PWM = 33;
const int RECOVERY_PWM = 24;
const int ALIGN_PWM_FAST = 31;
const int ALIGN_PWM_SLOW = 27;
const int TURN_PWM_FAST = 35;
const int TURN_PWM_SLOW = 28;
const int RIGHT_TURN_PWM_FAST = 32;
const int RIGHT_TURN_PWM_SLOW = 25;
const int HEADING_CORRECT_PWM = 25;
const int RIGHT_STRAFE_HEADING_PWM = 20;
const float LEFT_SIDE_TRIM = 1.10f;
const float FIRST_STAGE_LEFT_SIDE_TRIM = 1.20f;
const float LEFT_AFTER_TURN_TRIM = 1.20f;
const int PRETURN_FORWARD_PWM = 31;
const unsigned long PRETURN_FORWARD_MS = 400UL;

const uint16_t RED_GREEN_GAP_FOR_RED = 12;
const uint16_t RED_GREEN_GAP_FOR_GREEN = 3;
const uint8_t REQUIRED_COLOR_STREAK = 3;

struct DistancePair {
  float rightCm;
  float leftCm;
};

struct ColorReading {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
};

enum RoutineState {
  STATE_IDLE,
  STATE_APPROACH_25,
  STATE_APPROACH_25_ALIGN,
  STATE_APPROACH_25_SLOW,
  STATE_APPROACH_25_SETTLE,
  STATE_ALIGN_BEFORE_COLOR,
  STATE_STRAFE_TO_COLOR,
  STATE_RECENTER_FOR_RIGHT_SEARCH,
  STATE_COLOR_CONFIRM,
  STATE_PRETURN_FORWARD,
  STATE_PRETURN_ADJUST,
  STATE_TURN_90,
  STATE_APPROACH_9,
  STATE_APPROACH_9_SETTLE,
  STATE_FINAL_ALIGN,
  STATE_DONE,
  STATE_FAULT
};

enum DetectedColor {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_NO_SIGNAL
};

RoutineState currentState = STATE_IDLE;
unsigned long stateStartMs = 0;
unsigned long conditionStartMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastDistanceSampleMs = 0;
unsigned long lastColorSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;
unsigned long lastHeadingSampleMs = 0;

bool lastButtonPressed = false;
uint8_t invalidDistanceCount = 0;

DistancePair latestDistances = {-1.0f, -1.0f};
float latestAverageCm = -1.0f;
float latestDiffCm = 0.0f;

ColorReading latestColorReading = {0, 0, 0};
DetectedColor latestRawColor = COLOR_UNKNOWN;
DetectedColor stableColor = COLOR_UNKNOWN;
DetectedColor lastRawColor = COLOR_UNKNOWN;
uint8_t colorStreak = 0;
DetectedColor selectedColor = COLOR_UNKNOWN;
uint16_t confirmRedCount = 0;
uint16_t confirmGreenCount = 0;
uint16_t confirmUnknownCount = 0;
bool colorSearchRightPhase = false;
float colorSearchAlignedHeadingDeg = 0.0f;

float gyroZBias = 0.0f;
float headingDeg = 0.0f;
float referenceHeadingDeg = 0.0f;
float turnTargetHeadingDeg = 0.0f;
float latestGyroZDps = 0.0f;
bool mpuReady = false;
uint8_t gyroReadFailCount = 0;

const char *faultMessage = "";

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

int applyLeftTrim(int pwm) {
  return constrain(static_cast<int>(pwm * LEFT_SIDE_TRIM), 0, 255);
}

int applyCustomLeftTrim(int pwm, float trim) {
  return constrain(static_cast<int>(pwm * trim), 0, 255);
}

void moveForward(int pwm) {
  int leftPwm = applyLeftTrim(pwm);
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(pwm);
}

void moveForwardWithTrim(int pwm, float trim) {
  int leftPwm = applyCustomLeftTrim(pwm, trim);
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(pwm);
}

void moveForwardAfterTurn(int pwm) {
  float trim = selectedColor == COLOR_GREEN ? LEFT_AFTER_TURN_TRIM : LEFT_SIDE_TRIM;
  moveForwardWithTrim(pwm, trim);
}

void moveBackward(int pwm) {
  int leftPwm = applyLeftTrim(pwm);
  MOTORA_BACKOFF(leftPwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(leftPwm);
  MOTORD_FORWARD(pwm);
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
  pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);

  digitalWrite(ULTRASONIC_RIGHT_TRIG_PIN, LOW);
  digitalWrite(ULTRASONIC_LEFT_TRIG_PIN, LOW);
}

void setupButton() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
}

bool isStartButtonPressed() {
  return digitalRead(START_BUTTON_PIN) == LOW;
}

void setupColorSensor() {
  pinMode(COLOR_S0_PIN, OUTPUT);
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  pinMode(COLOR_LED_PIN, OUTPUT);

  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);
  digitalWrite(COLOR_LED_PIN, HIGH);
}

void setupOled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }

  oledReady = true;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("AI Robot");
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

float readGyroZDps() {
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
  Serial.println("Calibrating gyro. Keep the robot still.");
  delay(500);

  for (int i = 0; i < sampleCount; ++i) {
    sum += readGyroZDps();
    delay(4);
  }

  gyroZBias = sum / sampleCount;
  latestGyroZDps = 0.0f;
  headingDeg = 0.0f;
  referenceHeadingDeg = 0.0f;
  turnTargetHeadingDeg = 0.0f;
  lastHeadingUpdateUs = micros();

  Serial.print("Gyro bias(dps): ");
  Serial.println(gyroZBias, 4);
}

void updateHeading() {
  if (!mpuReady) {
    return;
  }

  if (millis() - lastHeadingSampleMs < HEADING_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastHeadingSampleMs = millis();

  unsigned long now = micros();
  if (lastHeadingUpdateUs == 0) {
    lastHeadingUpdateUs = now;
    return;
  }

  float dt = (now - lastHeadingUpdateUs) / 1000000.0f;
  lastHeadingUpdateUs = now;

  uint8_t buffer[2];
  if (!readMpuBytes(0x47, 2, buffer)) {
    if (gyroReadFailCount < 255) {
      gyroReadFailCount++;
    }
    if (gyroReadFailCount >= 5) {
      mpuReady = false;
      enterFault("MPU6050 read timeout");
    }
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
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
    if (duration != 0) {
      return duration * 0.0343f * 0.5f;
    }
    delay(2);
  }

  return -1.0f;
}

DistancePair readDistances() {
  DistancePair pair;
  pair.rightCm = readDistanceCm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
  delay(4);
  pair.leftCm = readDistanceCm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
  return pair;
}

void updateDistanceSnapshot() {
  if (!stateNeedsDistanceSampling(currentState)) {
    return;
  }

  if (millis() - lastDistanceSampleMs < DISTANCE_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastDistanceSampleMs = millis();
  latestDistances = readDistances();

  if (latestDistances.rightCm >= 0.0f && latestDistances.leftCm >= 0.0f) {
    latestAverageCm = (latestDistances.rightCm + latestDistances.leftCm) * 0.5f;
    latestDiffCm = latestDistances.rightCm - latestDistances.leftCm;
    invalidDistanceCount = 0;
  } else {
    latestAverageCm = -1.0f;
    latestDiffCm = 0.0f;
    if (invalidDistanceCount < 255) {
      invalidDistanceCount++;
    }
  }
}

uint16_t readColorChannel(bool s2, bool s3) {
  digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
  delay(3);

  unsigned long pulse = pulseIn(COLOR_OUT_PIN, LOW, 50000UL);
  if (pulse == 0) {
    return 0;
  }

  return static_cast<uint16_t>(pulse);
}

ColorReading readColor() {
  ColorReading reading;
  reading.red = readColorChannel(false, false);
  reading.blue = readColorChannel(false, true);
  reading.green = readColorChannel(true, true);
  return reading;
}

DetectedColor classifyColor(const ColorReading &reading) {
  if (reading.red == 0 || reading.green == 0 || reading.blue == 0) {
    return COLOR_NO_SIGNAL;
  }

  if (reading.red + RED_GREEN_GAP_FOR_RED < reading.green) {
    return COLOR_RED;
  }

  if (reading.green + RED_GREEN_GAP_FOR_GREEN < reading.red) {
    return COLOR_GREEN;
  }

  return COLOR_UNKNOWN;
}

void resetColorTracking() {
  latestColorReading.red = 0;
  latestColorReading.green = 0;
  latestColorReading.blue = 0;
  latestRawColor = COLOR_UNKNOWN;
  stableColor = COLOR_UNKNOWN;
  lastRawColor = COLOR_UNKNOWN;
  colorStreak = 0;
  confirmRedCount = 0;
  confirmGreenCount = 0;
  confirmUnknownCount = 0;
}

DetectedColor updateStableColor(DetectedColor rawColor) {
  if (rawColor == lastRawColor) {
    if (colorStreak < 255) {
      colorStreak++;
    }
  } else {
    lastRawColor = rawColor;
    colorStreak = 1;
  }

  if (colorStreak >= REQUIRED_COLOR_STREAK) {
    if (rawColor == COLOR_RED || rawColor == COLOR_GREEN) {
      stableColor = rawColor;
    } else if (stableColor == COLOR_UNKNOWN) {
      stableColor = rawColor;
    }
  }

  return stableColor;
}

bool stateNeedsColorSampling(RoutineState state) {
  return state == STATE_STRAFE_TO_COLOR || state == STATE_COLOR_CONFIRM;
}

bool stateNeedsHeadingUpdate(RoutineState state) {
  return state == STATE_RECENTER_FOR_RIGHT_SEARCH ||
         state == STATE_TURN_90 ||
         state == STATE_APPROACH_9 ||
         (state == STATE_STRAFE_TO_COLOR && colorSearchRightPhase);
}

bool stateNeedsDistanceSampling(RoutineState state) {
  return state != STATE_TURN_90;
}

void updateColorSnapshot() {
  if (!stateNeedsColorSampling(currentState)) {
    return;
  }

  if (millis() - lastColorSampleMs < COLOR_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastColorSampleMs = millis();
  latestColorReading = readColor();
  latestRawColor = classifyColor(latestColorReading);
  updateStableColor(latestRawColor);

  if (currentState == STATE_COLOR_CONFIRM) {
    if (latestRawColor == COLOR_RED) {
      confirmRedCount++;
    } else if (latestRawColor == COLOR_GREEN) {
      confirmGreenCount++;
    } else {
      confirmUnknownCount++;
    }
  }
}

const char *stateName(RoutineState state) {
  switch (state) {
    case STATE_IDLE: return "IDLE";
    case STATE_APPROACH_25: return "TO27";
    case STATE_APPROACH_25_ALIGN: return "A27ALN";
    case STATE_APPROACH_25_SLOW: return "TO27S";
    case STATE_APPROACH_25_SETTLE: return "CHK27";
    case STATE_ALIGN_BEFORE_COLOR: return "ALGN1";
    case STATE_STRAFE_TO_COLOR: return colorSearchRightPhase ? "RIGHT" : "LEFT";
    case STATE_RECENTER_FOR_RIGHT_SEARCH: return "RECEN";
    case STATE_COLOR_CONFIRM: return "COLOR";
    case STATE_PRETURN_FORWARD: return "PREFWD";
    case STATE_PRETURN_ADJUST: return "PRETRN";
    case STATE_TURN_90: return "TURN";
    case STATE_APPROACH_9: return "TO9";
    case STATE_APPROACH_9_SETTLE: return "CHK9";
    case STATE_FINAL_ALIGN: return "ALGN2";
    case STATE_DONE: return "DONE";
    case STATE_FAULT: return "FAULT";
    default: return "?";
  }
}

const char *colorName(DetectedColor color) {
  switch (color) {
    case COLOR_RED: return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_NO_SIGNAL: return "NONE";
    case COLOR_UNKNOWN:
    default:
      return "UNK";
  }
}

void setState(RoutineState newState) {
  currentState = newState;
  stateStartMs = millis();
  conditionStartMs = 0;

  if (newState == STATE_STRAFE_TO_COLOR) {
    if (!colorSearchRightPhase) {
      colorSearchAlignedHeadingDeg = headingDeg;
    }
    selectedColor = COLOR_UNKNOWN;
    resetColorTracking();
    lastColorSampleMs = 0;
  }

  Serial.print("State -> ");
  Serial.println(stateName(newState));
}

void enterFault(const char *message) {
  stopCar();
  faultMessage = message;
  setState(STATE_FAULT);
  Serial.print("FAULT: ");
  Serial.println(message);
}

void beginRun() {
  if (!mpuReady) {
    enterFault("MPU6050 not ready");
    return;
  }

  stopCar();
  headingDeg = 0.0f;
  referenceHeadingDeg = 0.0f;
  turnTargetHeadingDeg = 0.0f;
  lastHeadingUpdateUs = micros();
  selectedColor = COLOR_UNKNOWN;
  faultMessage = "";
  invalidDistanceCount = 0;
  colorSearchRightPhase = false;
  colorSearchAlignedHeadingDeg = 0.0f;
  lastDistanceSampleMs = 0;
  lastColorSampleMs = 0;
  lastHeadingSampleMs = 0;
  gyroReadFailCount = 0;
  resetColorTracking();
  setState(STATE_APPROACH_25);
}

int computeApproachPwm(float averageCm, float slowStartCm, float crawlStartCm,
                       int fastPwm, int slowPwm, int crawlPwm) {
  if (averageCm > slowStartCm) {
    return fastPwm;
  }
  if (averageCm > crawlStartCm) {
    return slowPwm;
  }
  return crawlPwm;
}

void runUltrasonicAlign(float toleranceCm, RoutineState nextState) {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout during align");
    }
    return;
  }

  float absDiff = fabs(latestDiffCm);
  if (absDiff <= toleranceCm) {
    stopCar();
    if (conditionStartMs == 0) {
      conditionStartMs = millis();
    } else if (millis() - conditionStartMs >= ALIGN_STABLE_MS) {
      setState(nextState);
    }
    return;
  }

  conditionStartMs = 0;
  int pwm = absDiff > 4.0f ? ALIGN_PWM_FAST : ALIGN_PWM_SLOW;
  if (latestDiffCm > 0.0f) {
    rotateLeft(pwm);
  } else {
    rotateRight(pwm);
  }
}

void runPreTurnAdjust() {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout before turn");
    }
    return;
  }

  float absDiff = fabs(latestDiffCm);

  if (absDiff > ALIGN_TOLERANCE_CM) {
    conditionStartMs = 0;
    int pwm = absDiff > 4.0f ? ALIGN_PWM_FAST : ALIGN_PWM_SLOW;
    if (latestDiffCm > 0.0f) {
      rotateLeft(pwm);
    } else {
      rotateRight(pwm);
    }
    return;
  }

  stopCar();
  if (conditionStartMs == 0) {
    conditionStartMs = millis();
  } else if (millis() - conditionStartMs >= ALIGN_STABLE_MS) {
    turnTargetHeadingDeg = headingDeg + (selectedColor == COLOR_GREEN ? TURN_ANGLE_DEG : -TURN_ANGLE_DEG);
    setState(STATE_TURN_90);
  }
}

void runRecenterForRightSearch() {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout before right search");
    }
    return;
  }

  float headingError = angleErrorDeg(colorSearchAlignedHeadingDeg, headingDeg);
  if (fabs(headingError) > HEADING_HOLD_TOLERANCE_DEG) {
    conditionStartMs = 0;
    if (headingError > 0.0f) {
      rotateLeft(HEADING_CORRECT_PWM);
    } else {
      rotateRight(HEADING_CORRECT_PWM);
    }
    return;
  }

  float absDiff = fabs(latestDiffCm);
  if (absDiff > ALIGN_TOLERANCE_CM) {
    conditionStartMs = 0;
    int pwm = absDiff > 4.0f ? ALIGN_PWM_FAST : ALIGN_PWM_SLOW;
    if (latestDiffCm > 0.0f) {
      rotateLeft(pwm);
    } else {
      rotateRight(pwm);
    }
    return;
  }

  float distanceError = latestAverageCm - FIRST_TARGET_CM;
  if (distanceError > PRETURN_DISTANCE_TOLERANCE_CM) {
    conditionStartMs = 0;
    moveForward(FORWARD_PWM_CRAWL);
    return;
  }

  if (distanceError < -PRETURN_DISTANCE_TOLERANCE_CM) {
    conditionStartMs = 0;
    moveBackward(RECOVERY_PWM);
    return;
  }

  stopCar();
  if (conditionStartMs == 0) {
    conditionStartMs = millis();
  } else if (millis() - conditionStartMs >= ALIGN_STABLE_MS) {
    colorSearchRightPhase = true;
    setState(STATE_STRAFE_TO_COLOR);
  }
}

void runApproach25() {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout before first stop");
    }
    return;
  }

  if (latestAverageCm <= FIRST_TARGET_CM + 0.6f) {
    stopCar();
    setState(STATE_APPROACH_25_SETTLE);
    return;
  }

  if (latestAverageCm <= FIRST_STAGE_TRANSITION_CM) {
    stopCar();
    setState(STATE_APPROACH_25_ALIGN);
    return;
  }

  moveForwardWithTrim(constrain(FORWARD_PWM_FAST * 2, 0, 255), FIRST_STAGE_LEFT_SIDE_TRIM);
}

void runApproach25Slow() {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout in slow approach");
    }
    return;
  }

  if (latestAverageCm <= FIRST_TARGET_CM + 0.6f) {
    stopCar();
    setState(STATE_APPROACH_25_SETTLE);
    return;
  }

  int pwm = computeApproachPwm(
    latestAverageCm,
    FIRST_SLOW_START_CM,
    FIRST_CRAWL_START_CM,
    FORWARD_PWM_FAST,
    FORWARD_PWM_SLOW,
    FORWARD_PWM_CRAWL
  );
  moveForward(pwm);
}

void runApproach25Settle() {
  if (millis() - stateStartMs < SETTLE_TIME_MS) {
    stopCar();
    return;
  }

  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout at first stop");
    }
    return;
  }

  if (latestAverageCm < FIRST_RECOVER_LOW_CM) {
    moveBackward(RECOVERY_PWM);
    return;
  }

  if (latestAverageCm > FIRST_RECOVER_HIGH_CM) {
    moveForward(FORWARD_PWM_CRAWL);
    return;
  }

  stopCar();
  referenceHeadingDeg = headingDeg;
  setState(STATE_ALIGN_BEFORE_COLOR);
}

void runStrafeToColor() {
  unsigned long elapsedMs = millis() - stateStartMs;
  if (!colorSearchRightPhase) {
    if (elapsedMs < COLOR_SEARCH_LEFT_MS) {
      strafeLeft(STRAFE_PWM);
    } else {
      stopCar();
      setState(STATE_RECENTER_FOR_RIGHT_SEARCH);
      return;
    }
  } else if (elapsedMs < COLOR_SEARCH_RIGHT_MS) {
    float headingError = angleErrorDeg(colorSearchAlignedHeadingDeg, headingDeg);
    if (fabs(headingError) > HEADING_HOLD_TOLERANCE_DEG) {
      if (headingError > 0.0f) {
        rotateLeft(RIGHT_STRAFE_HEADING_PWM);
      } else {
        rotateRight(RIGHT_STRAFE_HEADING_PWM);
      }
      return;
    }
    strafeRight(RIGHT_STRAFE_PWM);
  } else {
    enterFault("Color search timeout");
    return;
  }

  if (stableColor == COLOR_RED || stableColor == COLOR_GREEN) {
    stopCar();
    selectedColor = stableColor;
    confirmRedCount = 0;
    confirmGreenCount = 0;
    confirmUnknownCount = 0;
    setState(STATE_COLOR_CONFIRM);
  }
}

void runColorConfirm() {
  stopCar();

  if (millis() - stateStartMs < COLOR_CONFIRM_TIME_MS) {
    return;
  }

  if (confirmRedCount == 0 && confirmGreenCount == 0) {
    resetColorTracking();
    setState(STATE_STRAFE_TO_COLOR);
    return;
  }

  selectedColor = confirmRedCount >= confirmGreenCount ? COLOR_RED : COLOR_GREEN;
  setState(STATE_PRETURN_FORWARD);
}

void runPreTurnForward() {
  if (millis() - stateStartMs < PRETURN_FORWARD_MS) {
    moveForward(PRETURN_FORWARD_PWM);
    return;
  }

  stopCar();
  setState(STATE_PRETURN_ADJUST);
}

void runTurn90() {
  if (millis() - stateStartMs >= TURN_TIMEOUT_MS) {
    enterFault("90 degree turn timeout");
    return;
  }

  float errorDeg = angleErrorDeg(turnTargetHeadingDeg, headingDeg);
  float absError = fabs(errorDeg);

  if (absError <= TURN_TOLERANCE_DEG) {
    stopCar();
    if (conditionStartMs == 0) {
      conditionStartMs = millis();
    } else if (millis() - conditionStartMs >= TURN_STABLE_MS) {
      referenceHeadingDeg = headingDeg;
      setState(STATE_APPROACH_9);
    }
    return;
  }

  conditionStartMs = 0;
  int pwm = errorDeg > 0.0f
              ? (absError > 20.0f ? TURN_PWM_FAST : TURN_PWM_SLOW)
              : (absError > 20.0f ? RIGHT_TURN_PWM_FAST : RIGHT_TURN_PWM_SLOW);
  if (errorDeg > 0.0f) {
    rotateLeft(pwm);
  } else {
    rotateRight(pwm);
  }
}

void runApproach9() {
  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout before 9 cm");
    }
    return;
  }

  if (latestAverageCm <= FINAL_HARD_STOP_CM) {
    enterFault("Too close to wall");
    return;
  }

  float headingError = angleErrorDeg(referenceHeadingDeg, headingDeg);
  if (fabs(headingError) > HEADING_HOLD_TOLERANCE_DEG) {
    if (headingError > 0.0f) {
      rotateLeft(HEADING_CORRECT_PWM);
    } else {
      rotateRight(HEADING_CORRECT_PWM);
    }
    return;
  }

  if (latestAverageCm <= SECOND_TARGET_CM + 0.5f) {
    stopCar();
    setState(STATE_APPROACH_9_SETTLE);
    return;
  }

  int pwm = computeApproachPwm(
    latestAverageCm,
    SECOND_SLOW_START_CM,
    SECOND_CRAWL_START_CM,
    FINAL_FORWARD_PWM_FAST,
    FINAL_FORWARD_PWM_SLOW,
    FINAL_FORWARD_PWM_CRAWL
  );
  moveForwardAfterTurn(pwm);
}

void runApproach9Settle() {
  if (millis() - stateStartMs < SETTLE_TIME_MS) {
    stopCar();
    return;
  }

  if (latestAverageCm < 0.0f) {
    if (invalidDistanceCount >= 3) {
      enterFault("Ultrasonic timeout at 9 cm settle");
    }
    return;
  }

  if (latestAverageCm <= FINAL_HARD_STOP_CM) {
    enterFault("Too close after final stop");
    return;
  }

  if (latestAverageCm < SECOND_RECOVER_LOW_CM) {
    moveBackward(RECOVERY_PWM);
    return;
  }

  if (latestAverageCm > SECOND_RECOVER_HIGH_CM) {
    moveForwardAfterTurn(FINAL_FORWARD_PWM_CRAWL);
    return;
  }

  stopCar();
  setState(STATE_FINAL_ALIGN);
}

void runStateMachine() {
  switch (currentState) {
    case STATE_IDLE:
      stopCar();
      break;

    case STATE_APPROACH_25:
      runApproach25();
      break;

    case STATE_APPROACH_25_ALIGN:
      runUltrasonicAlign(ALIGN_TOLERANCE_CM, STATE_APPROACH_25_SLOW);
      break;

    case STATE_APPROACH_25_SLOW:
      runApproach25Slow();
      break;

    case STATE_APPROACH_25_SETTLE:
      runApproach25Settle();
      break;

    case STATE_ALIGN_BEFORE_COLOR:
      runUltrasonicAlign(ALIGN_TOLERANCE_CM, STATE_STRAFE_TO_COLOR);
      break;

    case STATE_STRAFE_TO_COLOR:
      runStrafeToColor();
      break;

    case STATE_RECENTER_FOR_RIGHT_SEARCH:
      runRecenterForRightSearch();
      break;

    case STATE_COLOR_CONFIRM:
      runColorConfirm();
      break;

    case STATE_PRETURN_FORWARD:
      runPreTurnForward();
      break;

    case STATE_PRETURN_ADJUST:
      runPreTurnAdjust();
      break;

    case STATE_TURN_90:
      runTurn90();
      break;

    case STATE_APPROACH_9:
      runApproach9();
      break;

    case STATE_APPROACH_9_SETTLE:
      runApproach9Settle();
      break;

    case STATE_FINAL_ALIGN:
      runUltrasonicAlign(FINAL_ALIGN_TOLERANCE_CM, STATE_DONE);
      break;

    case STATE_DONE:
      stopCar();
      break;

    case STATE_FAULT:
      stopCar();
      break;
  }
}

void printFloatOrDash(float value) {
  if (value < 0.0f) {
    display.print("--");
  } else {
    display.print(value, 1);
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

  if (currentState == STATE_FAULT) {
    display.println("FAULT");
    display.println(faultMessage);
    display.print("R:");
    printFloatOrDash(latestDistances.rightCm);
    display.print(" L:");
    printFloatOrDash(latestDistances.leftCm);
    display.display();
    return;
  }

  display.print(stateName(currentState));
  display.print(" C:");
  display.println(colorName(selectedColor != COLOR_UNKNOWN ? selectedColor : stableColor));

  display.print("R:");
  printFloatOrDash(latestDistances.rightCm);
  display.print(" L:");
  printFloatOrDash(latestDistances.leftCm);
  display.println();

  display.print("A:");
  printFloatOrDash(latestAverageCm);
  display.print(" D:");
  display.println(latestDiffCm, 1);

  display.print("H:");
  display.print(headingDeg, 1);
  display.display();
}

void printTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = millis();
  Serial.print("ms=");
  Serial.print(lastTelemetryMs);
  Serial.print(" state=");
  Serial.print(stateName(currentState));
  Serial.print(" right=");
  Serial.print(latestDistances.rightCm, 1);
  Serial.print(" left=");
  Serial.print(latestDistances.leftCm, 1);
  Serial.print(" avg=");
  Serial.print(latestAverageCm, 1);
  Serial.print(" diff=");
  Serial.print(latestDiffCm, 1);
  Serial.print(" heading=");
  Serial.print(headingDeg, 1);
  Serial.print(" gyroZ=");
  Serial.print(latestGyroZDps, 2);
  Serial.print(" rawColor=");
  Serial.print(colorName(latestRawColor));
  Serial.print(" stableColor=");
  Serial.print(colorName(stableColor));
  if (currentState == STATE_FAULT) {
    Serial.print(" fault=");
    Serial.print(faultMessage);
  }
  Serial.println();
}

void printHelp() {
  Serial.println("Final required-function sketch ready.");
  Serial.println("Commands:");
  Serial.println("  G - start routine");
  Serial.println("  X - stop and return to idle");
  Serial.println("  C - recalibrate gyro bias");
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'G':
    case 'g':
      if (currentState == STATE_IDLE || currentState == STATE_DONE || currentState == STATE_FAULT) {
        beginRun();
      }
      break;

    case 'X':
    case 'x':
      stopCar();
      selectedColor = COLOR_UNKNOWN;
      faultMessage = "";
      colorSearchRightPhase = false;
      resetColorTracking();
      setState(STATE_IDLE);
      break;

    case 'C':
    case 'c':
      if (!mpuReady) {
        enterFault("MPU6050 not ready");
        break;
      }
      stopCar();
      calibrateGyroBias();
      selectedColor = COLOR_UNKNOWN;
      colorSearchRightPhase = false;
      resetColorTracking();
      setState(STATE_IDLE);
      break;

    default:
      break;
  }
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
  setupColorSensor();
  setupOled();
  stopCar();

  if (!initMpu6050()) {
    Serial.println("MPU6050 init failed");
    faultMessage = "MPU6050 init failed";
    setState(STATE_FAULT);
  } else {
    mpuReady = true;
    calibrateGyroBias();
    setState(STATE_IDLE);
  }

  printHelp();
}

void loop() {
  if (stateNeedsHeadingUpdate(currentState)) {
    updateHeading();
  }
  updateDistanceSnapshot();
  updateColorSnapshot();

  bool buttonPressed = isStartButtonPressed();
  if (buttonPressed && !lastButtonPressed) {
    if (currentState == STATE_IDLE || currentState == STATE_DONE || currentState == STATE_FAULT) {
      beginRun();
    }
  }
  lastButtonPressed = buttonPressed;

  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  runStateMachine();
  updateOled();
  printTelemetry();
}
