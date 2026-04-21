#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 复用 Required Function 阶段已经验证过的电机引脚定义。
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

const uint8_t START_BUTTON_PIN = 33;
const uint8_t MPU6050_ADDR = 0x68;

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const int8_t OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const unsigned long TELEMETRY_INTERVAL_MS = 250UL;
const unsigned long OLED_INTERVAL_MS = 120UL;
const unsigned long OLED_PAGE_INTERVAL_MS = 1600UL;
const unsigned long HEADING_SAMPLE_INTERVAL_MS = 8UL;
const unsigned long BUTTON_DEBOUNCE_MS = 40UL;
const unsigned long RUN_PREPARE_MS = 450UL;
const unsigned long RUN_FINISH_BRAKE_MS = 200UL;

const float GYRO_SCALE_DPS_PER_LSB = 131.0f;
const float HEADING_HOLD_KP = 2.4f;
const float HEADING_HOLD_DEADBAND_DEG = 0.8f;
const int HEADING_HOLD_MAX_PWM = 28;

const uint8_t AUTO_PASS_LIMIT = 2;
const uint8_t SEARCH_PLAN_CAPACITY = 9;
const int MIN_PWM = 0;
const int MAX_PWM = 255;

const uint16_t EEPROM_BASE_ADDR = 0;
const uint32_t EEPROM_MAGIC = 0x43414C32UL;  // "CAL2"
const uint16_t EEPROM_VERSION = 2;

enum MotionMode {
  MODE_FORWARD = 0,
  MODE_BACKWARD,
  MODE_STRAFE_LEFT,
  MODE_STRAFE_RIGHT,
  MODE_COUNT
};

enum SpeedTier {
  TIER_LOW = 0,
  TIER_MEDIUM,
  TIER_HIGH,
  TIER_COUNT
};

enum MotorIndex {
  MOTOR_A = 0,
  MOTOR_B,
  MOTOR_C,
  MOTOR_D,
  MOTOR_COUNT
};

enum SessionType {
  SESSION_NONE = 0,
  SESSION_TEST_ONLY,
  SESSION_AUTO_SEARCH
};

enum SessionPhase {
  PHASE_IDLE = 0,
  PHASE_PREPARE,
  PHASE_RUNNING,
  PHASE_BRAKING,
  PHASE_FAIL
};

struct CalibrationProfile {
  uint8_t pwm[MOTOR_COUNT];
};

struct CalibrationBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint16_t checksum;
  CalibrationProfile profiles[MODE_COUNT][TIER_COUNT];
} __attribute__((packed));

struct RunMetrics {
  float finalHeadingDeg;
  float peakHeadingAbsDeg;
  float headingAbsArea;
  float correctionAbsArea;
  float meanCorrectionAbs;
  float score;
};

struct CandidatePlan {
  int8_t motorIndex;
  int8_t delta;
};

struct CalibrationOutcome {
  bool valid;
  MotionMode mode;
  SpeedTier tier;
  CalibrationProfile profile;
  RunMetrics metrics;
  bool cameFromAuto;
  bool needsManualReview;
  char note[48];
};

struct CalibrationSession {
  SessionType type;
  SessionPhase phase;
  MotionMode mode;
  SpeedTier tier;
  CalibrationProfile runProfile;
  CalibrationProfile searchCenterProfile;
  CalibrationProfile bestProfile;
  RunMetrics metrics;
  RunMetrics bestMetrics;
  CandidatePlan plan[SEARCH_PLAN_CAPACITY];
  uint8_t planCount;
  uint8_t planIndex;
  uint8_t passIndex;
  float bestScore;
  bool bestValid;
  int lastCorrectionPwm;
  unsigned long phaseStartMs;
  unsigned long lastMetricSampleMs;
  char failReason[48];
};

const int8_t DRIVE_SIGN[MODE_COUNT][MOTOR_COUNT] = {
  {1, -1, 1, -1},
  {-1, 1, -1, 1},
  {-1, -1, 1, 1},
  {1, 1, -1, -1}
};

const CalibrationProfile DEFAULT_PROFILE_TABLE[MODE_COUNT][TIER_COUNT] = {
  {
    {{92, 84, 92, 84}},
    {{106, 96, 106, 96}},
    {{123, 112, 123, 112}}
  },
  {
    {{92, 84, 92, 84}},
    {{106, 96, 106, 96}},
    {{123, 112, 123, 112}}
  },
  {
    {{82, 86, 82, 82}},
    {{96, 101, 96, 96}},
    {{110, 116, 110, 110}}
  },
  {
    {{82, 90, 90, 82}},
    {{96, 106, 106, 96}},
    {{110, 121, 121, 110}}
  }
};

CalibrationProfile profiles[MODE_COUNT][TIER_COUNT];
MotionMode selectedMode = MODE_FORWARD;
SpeedTier selectedTier = TIER_MEDIUM;

CalibrationSession session = {};
CalibrationOutcome lastOutcome = {};

bool profilesDirty = false;
bool oledReady = false;
bool mpuReady = false;
bool lastButtonState = false;
unsigned long lastButtonEdgeMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastHeadingSampleMs = 0;
unsigned long lastHeadingUpdateUs = 0;
unsigned long lastFlashStatusUntilMs = 0;

float gyroZBias = 0.0f;
float headingDeg = 0.0f;
float latestGyroZDps = 0.0f;
int currentWheelPwm[MOTOR_COUNT] = {0, 0, 0, 0};

char serialBuffer[80];
uint8_t serialLength = 0;
char lastError[48] = "";
char flashStatusLabel[12] = "";

const char *modeName(MotionMode mode) {
  switch (mode) {
    case MODE_FORWARD: return "FWD";
    case MODE_BACKWARD: return "BWD";
    case MODE_STRAFE_LEFT: return "SL";
    case MODE_STRAFE_RIGHT: return "SR";
    case MODE_COUNT:
    default:
      return "?";
  }
}

const char *tierName(SpeedTier tier) {
  switch (tier) {
    case TIER_LOW: return "LOW";
    case TIER_MEDIUM: return "MED";
    case TIER_HIGH: return "HIGH";
    case TIER_COUNT:
    default:
      return "?";
  }
}

const char *motorName(MotorIndex motor) {
  switch (motor) {
    case MOTOR_A: return "A";
    case MOTOR_B: return "B";
    case MOTOR_C: return "C";
    case MOTOR_D: return "D";
    case MOTOR_COUNT:
    default:
      return "?";
  }
}

void clearError() {
  lastError[0] = '\0';
}

void invalidateLastOutcome() {
  lastOutcome.valid = false;
}

void setError(const char *message) {
  strncpy(lastError, message, sizeof(lastError) - 1);
  lastError[sizeof(lastError) - 1] = '\0';
}

void flashStatus(const char *label, unsigned long durationMs) {
  strncpy(flashStatusLabel, label, sizeof(flashStatusLabel) - 1);
  flashStatusLabel[sizeof(flashStatusLabel) - 1] = '\0';
  lastFlashStatusUntilMs = millis() + durationMs;
}

bool sessionIsActive() {
  return session.phase == PHASE_PREPARE ||
         session.phase == PHASE_RUNNING ||
         session.phase == PHASE_BRAKING;
}

bool modeIsSemiAuto(MotionMode mode) {
  return mode == MODE_STRAFE_LEFT || mode == MODE_STRAFE_RIGHT;
}

int clampPwm(int pwm) {
  return constrain(pwm, MIN_PWM, MAX_PWM);
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
  int pwm = clampPwm(abs(signedPwm));
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
  session.lastCorrectionPwm = 0;
}

void driveProfileWithHeadingHold(MotionMode mode, const CalibrationProfile &profile) {
  int averagePwm = 0;
  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    averagePwm += profile.pwm[motor];
  }
  averagePwm /= MOTOR_COUNT;

  float headingErrorDeg = -headingDeg;
  int maxCorrection = min(HEADING_HOLD_MAX_PWM, max(averagePwm / 3, 8));
  int correction = 0;
  if (fabs(headingErrorDeg) > HEADING_HOLD_DEADBAND_DEG) {
    correction = constrain(static_cast<int>(headingErrorDeg * HEADING_HOLD_KP),
                           -maxCorrection,
                           maxCorrection);
  }
  session.lastCorrectionPwm = correction;

  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    int base = profile.pwm[motor];
    int signedPwm = DRIVE_SIGN[mode][motor] * base - correction;
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

void setupButton() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
}

bool isStartButtonPressed() {
  return digitalRead(START_BUTTON_PIN) == LOW;
}

void setupOled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }

  oledReady = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Motor Calib"));
  display.println(F("Gyro only"));
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
  Serial.println(F("Calibrating gyro. Keep robot still."));
  delay(500);

  for (int i = 0; i < sampleCount; ++i) {
    sum += readGyroZDps();
    delay(4);
  }

  gyroZBias = sum / sampleCount;
  latestGyroZDps = 0.0f;
  headingDeg = 0.0f;
  lastHeadingSampleMs = 0;
  lastHeadingUpdateUs = 0;

  Serial.print(F("Gyro bias(dps): "));
  Serial.println(gyroZBias, 4);
}

void zeroRelativeHeading() {
  headingDeg = 0.0f;
  latestGyroZDps = 0.0f;
  lastHeadingSampleMs = 0;
  lastHeadingUpdateUs = 0;
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
    setError("MPU6050 read fail");
    latestGyroZDps = 0.0f;
    return;
  }

  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  float gyroZ = static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB - gyroZBias;
  latestGyroZDps = gyroZ;
  headingDeg += gyroZ * dt;
}

unsigned long runDurationForTier(SpeedTier tier) {
  switch (tier) {
    case TIER_LOW: return 2200UL;
    case TIER_MEDIUM: return 1800UL;
    case TIER_HIGH: return 1500UL;
    case TIER_COUNT:
    default:
      return 1800UL;
  }
}

int8_t searchDeltaForTier(SpeedTier tier) {
  switch (tier) {
    case TIER_LOW: return 2;
    case TIER_MEDIUM: return 3;
    case TIER_HIGH: return 4;
    case TIER_COUNT:
    default:
      return 3;
  }
}

void restoreDefaultProfiles() {
  for (uint8_t mode = 0; mode < MODE_COUNT; ++mode) {
    for (uint8_t tier = 0; tier < TIER_COUNT; ++tier) {
      profiles[mode][tier] = DEFAULT_PROFILE_TABLE[mode][tier];
    }
  }
  profilesDirty = true;
}

uint16_t computeBlobChecksum(CalibrationBlob blob) {
  blob.checksum = 0;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&blob);
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < sizeof(CalibrationBlob); ++i) {
    checksum = static_cast<uint16_t>((checksum * 131U) + bytes[i]);
  }
  return checksum;
}

bool loadProfilesFromEeprom() {
  CalibrationBlob blob;
  EEPROM.get(EEPROM_BASE_ADDR, blob);

  if (blob.magic != EEPROM_MAGIC) {
    return false;
  }
  if (blob.version != EEPROM_VERSION) {
    return false;
  }
  if (blob.payloadSize != sizeof(blob.profiles)) {
    return false;
  }
  if (blob.checksum != computeBlobChecksum(blob)) {
    return false;
  }

  for (uint8_t mode = 0; mode < MODE_COUNT; ++mode) {
    for (uint8_t tier = 0; tier < TIER_COUNT; ++tier) {
      profiles[mode][tier] = blob.profiles[mode][tier];
    }
  }

  profilesDirty = false;
  return true;
}

bool saveProfilesToEeprom() {
  CalibrationBlob blob;
  blob.magic = EEPROM_MAGIC;
  blob.version = EEPROM_VERSION;
  blob.payloadSize = sizeof(blob.profiles);
  blob.checksum = 0;

  for (uint8_t mode = 0; mode < MODE_COUNT; ++mode) {
    for (uint8_t tier = 0; tier < TIER_COUNT; ++tier) {
      blob.profiles[mode][tier] = profiles[mode][tier];
    }
  }

  blob.checksum = computeBlobChecksum(blob);
  EEPROM.put(EEPROM_BASE_ADDR, blob);
  profilesDirty = false;
  return true;
}

bool profilesEqual(const CalibrationProfile &lhs, const CalibrationProfile &rhs) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (lhs.pwm[i] != rhs.pwm[i]) {
      return false;
    }
  }
  return true;
}

CalibrationProfile applyCandidatePlan(const CalibrationProfile &center, const CandidatePlan &plan) {
  CalibrationProfile candidate = center;
  if (plan.motorIndex >= 0 && plan.motorIndex < MOTOR_COUNT) {
    int pwm = static_cast<int>(candidate.pwm[plan.motorIndex]) + plan.delta;
    candidate.pwm[plan.motorIndex] = static_cast<uint8_t>(clampPwm(pwm));
  }
  return candidate;
}

void clearSession() {
  session.type = SESSION_NONE;
  session.phase = PHASE_IDLE;
  session.planCount = 0;
  session.planIndex = 0;
  session.passIndex = 0;
  session.bestScore = 0.0f;
  session.bestValid = false;
  session.lastCorrectionPwm = 0;
  session.phaseStartMs = 0;
  session.lastMetricSampleMs = 0;
  session.failReason[0] = '\0';
}

void setSessionFailure(const char *reason) {
  stopCar();
  session.phase = PHASE_FAIL;
  session.type = SESSION_NONE;
  strncpy(session.failReason, reason, sizeof(session.failReason) - 1);
  session.failReason[sizeof(session.failReason) - 1] = '\0';
  setError(reason);
  flashStatus("FAIL", 1500UL);
  Serial.print(F("FAIL: "));
  Serial.println(reason);
}

void resetMetrics(RunMetrics &metrics) {
  metrics.finalHeadingDeg = 0.0f;
  metrics.peakHeadingAbsDeg = 0.0f;
  metrics.headingAbsArea = 0.0f;
  metrics.correctionAbsArea = 0.0f;
  metrics.meanCorrectionAbs = 0.0f;
  metrics.score = 0.0f;
}

float computeScore(const RunMetrics &metrics) {
  return fabs(metrics.finalHeadingDeg) * 8.0f +
         metrics.peakHeadingAbsDeg * 4.0f +
         metrics.headingAbsArea * 1.2f +
         metrics.meanCorrectionAbs * 1.5f;
}

bool selectedProfileNeedsManualReview(MotionMode mode) {
  return modeIsSemiAuto(mode);
}

void recordOutcome(MotionMode mode,
                   SpeedTier tier,
                   const CalibrationProfile &profile,
                   const RunMetrics &metrics,
                   bool cameFromAuto,
                   bool needsManualReview,
                   const char *note) {
  lastOutcome.valid = true;
  lastOutcome.mode = mode;
  lastOutcome.tier = tier;
  lastOutcome.profile = profile;
  lastOutcome.metrics = metrics;
  lastOutcome.cameFromAuto = cameFromAuto;
  lastOutcome.needsManualReview = needsManualReview;
  strncpy(lastOutcome.note, note, sizeof(lastOutcome.note) - 1);
  lastOutcome.note[sizeof(lastOutcome.note) - 1] = '\0';
}

void printProfileLine(MotionMode mode, SpeedTier tier, const CalibrationProfile &profile) {
  Serial.print(modeName(mode));
  Serial.print(' ');
  Serial.print(tierName(tier));
  Serial.print(F(" -> "));
  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    Serial.print(motorName(static_cast<MotorIndex>(motor)));
    Serial.print('=');
    Serial.print(profile.pwm[motor]);
    if (motor + 1 < MOTOR_COUNT) {
      Serial.print(' ');
    }
  }
  if (modeIsSemiAuto(mode)) {
    Serial.print(F(" [semi-auto]"));
  }
  Serial.println();
}

void printSelectedProfile() {
  printProfileLine(selectedMode, selectedTier, profiles[selectedMode][selectedTier]);
}

void printAllProfiles() {
  for (uint8_t mode = 0; mode < MODE_COUNT; ++mode) {
    for (uint8_t tier = 0; tier < TIER_COUNT; ++tier) {
      printProfileLine(static_cast<MotionMode>(mode),
                       static_cast<SpeedTier>(tier),
                       profiles[mode][tier]);
    }
  }
}

void printLastOutcome() {
  if (!lastOutcome.valid) {
    Serial.println(F("No last outcome."));
    return;
  }

  Serial.print(F("Last "));
  Serial.print(lastOutcome.cameFromAuto ? F("AUTO ") : F("TEST "));
  Serial.print(modeName(lastOutcome.mode));
  Serial.print(' ');
  Serial.print(tierName(lastOutcome.tier));
  Serial.print(F(" note="));
  Serial.print(lastOutcome.note);
  Serial.print(F(" score="));
  Serial.print(lastOutcome.metrics.score, 2);
  Serial.print(F(" heading="));
  Serial.print(lastOutcome.metrics.finalHeadingDeg, 2);
  Serial.print(F(" peak="));
  Serial.print(lastOutcome.metrics.peakHeadingAbsDeg, 2);
  Serial.print(F(" meanCorr="));
  Serial.print(lastOutcome.metrics.meanCorrectionAbs, 2);
  if (lastOutcome.needsManualReview) {
    Serial.print(F(" review=YES"));
  }
  Serial.println();

  printProfileLine(lastOutcome.mode, lastOutcome.tier, lastOutcome.profile);
}

void buildSearchPlan(uint8_t passIndex) {
  session.planCount = 0;
  session.planIndex = 0;

  session.plan[session.planCount++] = {-1, 0};

  int8_t delta = searchDeltaForTier(session.tier) - static_cast<int8_t>(passIndex);
  if (delta < 1) {
    delta = 1;
  }

  for (uint8_t motor = 0; motor < MOTOR_COUNT && session.planCount + 1 < SEARCH_PLAN_CAPACITY; ++motor) {
    session.plan[session.planCount++] = {static_cast<int8_t>(motor), static_cast<int8_t>(-delta)};
    session.plan[session.planCount++] = {static_cast<int8_t>(motor), delta};
  }
}

void startCandidateRun() {
  if (session.planIndex >= session.planCount) {
    setSessionFailure("Candidate plan overflow");
    return;
  }

  session.runProfile = applyCandidatePlan(session.searchCenterProfile, session.plan[session.planIndex]);
  resetMetrics(session.metrics);
  session.lastCorrectionPwm = 0;
  session.phase = PHASE_PREPARE;
  session.phaseStartMs = millis();
  session.lastMetricSampleMs = 0;

  Serial.print(F("Candidate "));
  Serial.print(session.planIndex + 1);
  Serial.print('/');
  Serial.print(session.planCount);
  Serial.print(F(" pass="));
  Serial.print(session.passIndex + 1);
  Serial.print(F(" -> "));
  for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
    Serial.print(motorName(static_cast<MotorIndex>(motor)));
    Serial.print('=');
    Serial.print(session.runProfile.pwm[motor]);
    if (motor + 1 < MOTOR_COUNT) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void startTestSession(bool autoSearch) {
  if (sessionIsActive()) {
    Serial.println(F("Session already running."));
    return;
  }
  if (!mpuReady) {
    setSessionFailure("MPU6050 not ready");
    return;
  }

  clearError();
  session.mode = selectedMode;
  session.tier = selectedTier;
  session.searchCenterProfile = profiles[selectedMode][selectedTier];
  session.passIndex = 0;
  session.planIndex = 0;
  session.bestValid = false;
  session.bestScore = 0.0f;
  session.type = autoSearch ? SESSION_AUTO_SEARCH : SESSION_TEST_ONLY;

  if (autoSearch) {
    buildSearchPlan(0);
  } else {
    session.planCount = 1;
    session.plan[0] = {-1, 0};
  }

  if (autoSearch && modeIsSemiAuto(selectedMode)) {
    Serial.println(F("Semi-auto mode: strafe search only minimizes yaw drift."));
  }

  startCandidateRun();
}

void updateRunMetrics() {
  if (session.phase != PHASE_RUNNING) {
    return;
  }

  unsigned long nowMs = millis();
  if (session.lastMetricSampleMs == 0) {
    session.lastMetricSampleMs = nowMs;
  }
  unsigned long dtMs = nowMs - session.lastMetricSampleMs;
  session.lastMetricSampleMs = nowMs;

  float absHeading = fabs(headingDeg);
  session.metrics.finalHeadingDeg = headingDeg;
  if (absHeading > session.metrics.peakHeadingAbsDeg) {
    session.metrics.peakHeadingAbsDeg = absHeading;
  }
  session.metrics.headingAbsArea += absHeading * (dtMs / 1000.0f);
  session.metrics.correctionAbsArea += fabs(static_cast<float>(session.lastCorrectionPwm)) * (dtMs / 1000.0f);
}

void finishCandidateRun() {
  stopCar();

  float runDurationSec = runDurationForTier(session.tier) / 1000.0f;
  if (runDurationSec > 0.0f) {
    session.metrics.meanCorrectionAbs = session.metrics.correctionAbsArea / runDurationSec;
  }
  session.metrics.score = computeScore(session.metrics);

  Serial.print(F("Candidate score="));
  Serial.print(session.metrics.score, 2);
  Serial.print(F(" heading="));
  Serial.print(session.metrics.finalHeadingDeg, 2);
  Serial.print(F(" peak="));
  Serial.print(session.metrics.peakHeadingAbsDeg, 2);
  Serial.print(F(" meanCorr="));
  Serial.println(session.metrics.meanCorrectionAbs, 2);

  if (session.type == SESSION_TEST_ONLY) {
    recordOutcome(session.mode,
                  session.tier,
                  session.runProfile,
                  session.metrics,
                  false,
                  selectedProfileNeedsManualReview(session.mode),
                  modeIsSemiAuto(session.mode) ? "manual strafe test" : "manual gyro test");
    flashStatus(lastOutcome.needsManualReview ? "REVIEW" : "DONE", 1500UL);
    clearSession();
    return;
  }

  if (!session.bestValid || session.metrics.score < session.bestScore) {
    session.bestValid = true;
    session.bestScore = session.metrics.score;
    session.bestProfile = session.runProfile;
    session.bestMetrics = session.metrics;
  }

  session.planIndex++;
  if (session.planIndex < session.planCount) {
    startCandidateRun();
    return;
  }

  bool centerChanged = !profilesEqual(session.searchCenterProfile, session.bestProfile);
  if (centerChanged && session.passIndex + 1 < AUTO_PASS_LIMIT) {
    session.searchCenterProfile = session.bestProfile;
    session.passIndex++;
    buildSearchPlan(session.passIndex);
    startCandidateRun();
    return;
  }

  profiles[session.mode][session.tier] = session.bestProfile;
  profilesDirty = true;

  bool needsManualReview = selectedProfileNeedsManualReview(session.mode);
  const char *note = needsManualReview ? "heading-only initial value" : "gyro auto";
  session.bestMetrics.score = session.bestScore;

  recordOutcome(session.mode,
                session.tier,
                session.bestProfile,
                session.bestMetrics,
                true,
                needsManualReview,
                note);

  Serial.print(F("Auto result saved in RAM for "));
  Serial.print(modeName(session.mode));
  Serial.print(' ');
  Serial.print(tierName(session.tier));
  Serial.print(F(" -> score="));
  Serial.println(session.bestScore, 2);

  flashStatus(needsManualReview ? "REVIEW" : "DONE", 1800UL);
  clearSession();
}

void updateCalibrationSession() {
  if (session.phase == PHASE_IDLE || session.phase == PHASE_FAIL) {
    return;
  }

  unsigned long nowMs = millis();

  switch (session.phase) {
    case PHASE_PREPARE:
      stopCar();
      if (nowMs - session.phaseStartMs >= RUN_PREPARE_MS) {
        zeroRelativeHeading();
        resetMetrics(session.metrics);
        session.phase = PHASE_RUNNING;
        session.phaseStartMs = nowMs;
        session.lastMetricSampleMs = nowMs;
      }
      break;

    case PHASE_RUNNING:
      driveProfileWithHeadingHold(session.mode, session.runProfile);
      updateRunMetrics();
      if (nowMs - session.phaseStartMs >= runDurationForTier(session.tier)) {
        stopCar();
        session.phase = PHASE_BRAKING;
        session.phaseStartMs = nowMs;
      }
      break;

    case PHASE_BRAKING:
      stopCar();
      if (nowMs - session.phaseStartMs >= RUN_FINISH_BRAKE_MS) {
        finishCandidateRun();
      }
      break;

    case PHASE_FAIL:
    case PHASE_IDLE:
    default:
      break;
  }
}

const char *currentStageName() {
  if (session.phase == PHASE_PREPARE) {
    return "SETTLE";
  }
  if (session.phase == PHASE_RUNNING) {
    return session.type == SESSION_AUTO_SEARCH ? "AUTO" : "TEST";
  }
  if (session.phase == PHASE_BRAKING) {
    return "STOP";
  }
  if (lastFlashStatusUntilMs > millis() && flashStatusLabel[0] != '\0') {
    return flashStatusLabel;
  }
  if (lastError[0] != '\0') {
    return "FAIL";
  }
  if (lastOutcome.valid &&
      lastOutcome.mode == selectedMode &&
      lastOutcome.tier == selectedTier) {
    return lastOutcome.needsManualReview ? "REVIEW" : "DONE";
  }
  return "IDLE";
}

void updateOled() {
  if (!oledReady) {
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastOledMs < OLED_INTERVAL_MS) {
    return;
  }
  lastOledMs = nowMs;

  const CalibrationProfile &profile = profiles[selectedMode][selectedTier];
  const RunMetrics &metrics = lastOutcome.valid &&
                              lastOutcome.mode == selectedMode &&
                              lastOutcome.tier == selectedTier
                                ? lastOutcome.metrics
                                : session.metrics;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(modeName(selectedMode));
  display.print(' ');
  display.print(tierName(selectedTier));
  display.print(' ');
  display.println(currentStageName());

  if (lastError[0] != '\0') {
    display.println(lastError);
    display.print(F("H:"));
    display.print(headingDeg, 1);
    display.print(F(" G:"));
    display.println(latestGyroZDps, 1);
    display.display();
    return;
  }

  uint8_t page = (nowMs / OLED_PAGE_INTERVAL_MS) % 2;
  if (page == 0) {
    display.print(F("A"));
    display.print(profile.pwm[MOTOR_A]);
    display.print(F(" B"));
    display.println(profile.pwm[MOTOR_B]);

    display.print(F("C"));
    display.print(profile.pwm[MOTOR_C]);
    display.print(F(" D"));
    display.println(profile.pwm[MOTOR_D]);

    if (session.type == SESSION_AUTO_SEARCH && sessionIsActive()) {
      display.print(F("P"));
      display.print(session.passIndex + 1);
      display.print(F(" "));
      display.print(session.planIndex + 1);
      display.print('/');
      display.println(session.planCount);
    } else {
      display.print(F("Dirty:"));
      display.println(profilesDirty ? F("YES") : F("NO"));
    }
  } else {
    display.print(F("H:"));
    display.print(metrics.finalHeadingDeg, 1);
    display.print(F(" P:"));
    display.println(metrics.peakHeadingAbsDeg, 1);

    display.print(F("Corr:"));
    display.print(metrics.meanCorrectionAbs, 1);
    display.print(F(" G:"));
    display.println(latestGyroZDps, 1);

    display.print(F("S:"));
    display.print(metrics.score, 1);
    display.print(F(" "));
    display.println(modeIsSemiAuto(selectedMode) ? F("semi") : F("auto"));
  }

  display.display();
}

void printTelemetry() {
  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = nowMs;

  const CalibrationProfile &profile = profiles[selectedMode][selectedTier];
  const RunMetrics &metrics = lastOutcome.valid &&
                              lastOutcome.mode == selectedMode &&
                              lastOutcome.tier == selectedTier
                                ? lastOutcome.metrics
                                : session.metrics;

  Serial.print(F("ms="));
  Serial.print(nowMs);
  Serial.print(F(" mode="));
  Serial.print(modeName(selectedMode));
  Serial.print(F(" tier="));
  Serial.print(tierName(selectedTier));
  Serial.print(F(" stage="));
  Serial.print(currentStageName());
  Serial.print(F(" A="));
  Serial.print(profile.pwm[MOTOR_A]);
  Serial.print(F(" B="));
  Serial.print(profile.pwm[MOTOR_B]);
  Serial.print(F(" C="));
  Serial.print(profile.pwm[MOTOR_C]);
  Serial.print(F(" D="));
  Serial.print(profile.pwm[MOTOR_D]);
  Serial.print(F(" heading="));
  Serial.print(headingDeg, 2);
  Serial.print(F(" gyroZ="));
  Serial.print(latestGyroZDps, 2);
  Serial.print(F(" corr="));
  Serial.print(session.lastCorrectionPwm);
  Serial.print(F(" score="));
  Serial.print(metrics.score, 2);
  if (lastOutcome.valid &&
      lastOutcome.mode == selectedMode &&
      lastOutcome.tier == selectedTier) {
    Serial.print(F(" note="));
    Serial.print(lastOutcome.note);
  }
  if (lastError[0] != '\0') {
    Serial.print(F(" err="));
    Serial.print(lastError);
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("Motor calibration assistant ready."));
  Serial.println(F("No side ultrasonic is used in this sketch."));
  Serial.println(F("FWD/BWD auto search: gyro heading hold based."));
  Serial.println(F("SL/SR auto search: semi-auto, heading-only initial value."));
  Serial.println(F("Commands (line ending recommended):"));
  Serial.println(F("  HELP"));
  Serial.println(F("  MODE FWD|BWD|SL|SR"));
  Serial.println(F("  TIER LOW|MED|HIGH"));
  Serial.println(F("  AUTO / CAL / START   -> run auto or semi-auto calibration"));
  Serial.println(F("  TEST / RUN           -> run one test with current profile"));
  Serial.println(F("  STOP / X             -> stop current run"));
  Serial.println(F("  SAVE                 -> save all profiles to EEPROM"));
  Serial.println(F("  LOAD                 -> load profiles from EEPROM"));
  Serial.println(F("  DEFAULTS             -> restore default profiles in RAM"));
  Serial.println(F("  PRINT [ALL|LAST]"));
  Serial.println(F("  SET A|B|C|D <PWM>"));
  Serial.println(F("  BUMP A|B|C|D <DELTA>"));
  Serial.println(F("  GYRO                 -> recalibrate gyro bias"));
  Serial.println(F("Button 33: idle=start auto, running=stop"));
}

MotionMode parseModeToken(const char *token, bool *ok) {
  if (strcmp(token, "FWD") == 0 || strcmp(token, "FORWARD") == 0) {
    *ok = true;
    return MODE_FORWARD;
  }
  if (strcmp(token, "BWD") == 0 || strcmp(token, "BACKWARD") == 0) {
    *ok = true;
    return MODE_BACKWARD;
  }
  if (strcmp(token, "SL") == 0 || strcmp(token, "STRAFELEFT") == 0 || strcmp(token, "STRAFE_LEFT") == 0) {
    *ok = true;
    return MODE_STRAFE_LEFT;
  }
  if (strcmp(token, "SR") == 0 || strcmp(token, "STRAFERIGHT") == 0 || strcmp(token, "STRAFE_RIGHT") == 0) {
    *ok = true;
    return MODE_STRAFE_RIGHT;
  }

  *ok = false;
  return MODE_FORWARD;
}

SpeedTier parseTierToken(const char *token, bool *ok) {
  if (strcmp(token, "LOW") == 0) {
    *ok = true;
    return TIER_LOW;
  }
  if (strcmp(token, "MED") == 0 || strcmp(token, "MEDIUM") == 0) {
    *ok = true;
    return TIER_MEDIUM;
  }
  if (strcmp(token, "HIGH") == 0) {
    *ok = true;
    return TIER_HIGH;
  }

  *ok = false;
  return TIER_MEDIUM;
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

void stopCurrentRun(const char *reason) {
  stopCar();
  clearSession();
  clearError();
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(F("Stop: "));
    Serial.println(reason);
  }
}

void uppercaseInPlace(char *text) {
  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    text[i] = static_cast<char>(toupper(static_cast<unsigned char>(text[i])));
  }
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

  if (strcmp(tokens[0], "MODE") == 0 && tokenCount >= 2) {
    if (sessionIsActive()) {
      Serial.println(F("Cannot change mode while running."));
      return;
    }
    bool ok = false;
    MotionMode mode = parseModeToken(tokens[1], &ok);
    if (!ok) {
      Serial.println(F("Bad mode token."));
      return;
    }
    selectedMode = mode;
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "TIER") == 0 && tokenCount >= 2) {
    if (sessionIsActive()) {
      Serial.println(F("Cannot change tier while running."));
      return;
    }
    bool ok = false;
    SpeedTier tier = parseTierToken(tokens[1], &ok);
    if (!ok) {
      Serial.println(F("Bad tier token."));
      return;
    }
    selectedTier = tier;
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "AUTO") == 0 || strcmp(tokens[0], "CAL") == 0 || strcmp(tokens[0], "START") == 0) {
    startTestSession(true);
    return;
  }

  if (strcmp(tokens[0], "TEST") == 0 || strcmp(tokens[0], "RUN") == 0) {
    startTestSession(false);
    return;
  }

  if (strcmp(tokens[0], "STOP") == 0 || strcmp(tokens[0], "X") == 0) {
    stopCurrentRun("user");
    flashStatus("STOP", 800UL);
    return;
  }

  if (strcmp(tokens[0], "SAVE") == 0) {
    clearError();
    saveProfilesToEeprom();
    flashStatus("SAVE", 1000UL);
    Serial.println(F("Profiles saved to EEPROM."));
    return;
  }

  if (strcmp(tokens[0], "LOAD") == 0) {
    if (loadProfilesFromEeprom()) {
      clearError();
      invalidateLastOutcome();
      flashStatus("LOAD", 1000UL);
      Serial.println(F("Profiles loaded from EEPROM."));
      printSelectedProfile();
    } else {
      Serial.println(F("EEPROM invalid. Keep current RAM values."));
    }
    return;
  }

  if (strcmp(tokens[0], "DEFAULTS") == 0) {
    clearError();
    restoreDefaultProfiles();
    invalidateLastOutcome();
    flashStatus("DONE", 1000UL);
    Serial.println(F("Default profiles restored in RAM."));
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "PRINT") == 0) {
    if (tokenCount >= 2 && strcmp(tokens[1], "ALL") == 0) {
      printAllProfiles();
      return;
    }
    if (tokenCount >= 2 && strcmp(tokens[1], "LAST") == 0) {
      printLastOutcome();
      return;
    }
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "SET") == 0 && tokenCount >= 3) {
    if (sessionIsActive()) {
      Serial.println(F("Cannot edit PWM while running."));
      return;
    }
    bool ok = false;
    MotorIndex motor = parseMotorToken(tokens[1], &ok);
    if (!ok) {
      Serial.println(F("Bad motor token."));
      return;
    }
    int pwm = clampPwm(atoi(tokens[2]));
    profiles[selectedMode][selectedTier].pwm[motor] = static_cast<uint8_t>(pwm);
    profilesDirty = true;
    invalidateLastOutcome();
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "BUMP") == 0 && tokenCount >= 3) {
    if (sessionIsActive()) {
      Serial.println(F("Cannot edit PWM while running."));
      return;
    }
    bool ok = false;
    MotorIndex motor = parseMotorToken(tokens[1], &ok);
    if (!ok) {
      Serial.println(F("Bad motor token."));
      return;
    }
    int pwm = profiles[selectedMode][selectedTier].pwm[motor] + atoi(tokens[2]);
    profiles[selectedMode][selectedTier].pwm[motor] = static_cast<uint8_t>(clampPwm(pwm));
    profilesDirty = true;
    invalidateLastOutcome();
    printSelectedProfile();
    return;
  }

  if (strcmp(tokens[0], "GYRO") == 0 || strcmp(tokens[0], "C") == 0) {
    if (!mpuReady) {
      Serial.println(F("MPU6050 not ready."));
      return;
    }
    calibrateGyroBias();
    flashStatus("DONE", 1000UL);
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

    if (serialLength == 0 &&
        Serial.available() == 0 &&
        (ch == 'x' || ch == 'X' || ch == 'c' || ch == 'C')) {
      char temp[2] = {ch, '\0'};
      handleCommandLine(temp);
      continue;
    }

    if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = ch;
    }
  }
}

void handleButtonInput() {
  bool pressed = isStartButtonPressed();
  unsigned long nowMs = millis();

  if (pressed != lastButtonState && nowMs - lastButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
    lastButtonEdgeMs = nowMs;
    lastButtonState = pressed;

    if (pressed) {
      if (sessionIsActive()) {
        stopCurrentRun("button");
        flashStatus("STOP", 800UL);
      } else {
        startTestSession(true);
      }
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
  setupButton();
  setupOled();
  stopCar();
  clearSession();

  if (!initMpu6050()) {
    mpuReady = false;
    setError("MPU6050 init fail");
    Serial.println(F("MPU6050 init failed"));
  } else {
    mpuReady = true;
    calibrateGyroBias();
  }

  if (!loadProfilesFromEeprom()) {
    restoreDefaultProfiles();
    Serial.println(F("EEPROM invalid, loaded factory defaults."));
  } else {
    Serial.println(F("Loaded profiles from EEPROM."));
  }

  printHelp();
  printSelectedProfile();
}

void loop() {
  updateHeading();
  processSerialInput();
  handleButtonInput();
  updateCalibrationSession();
  updateOled();
  printTelemetry();
}
