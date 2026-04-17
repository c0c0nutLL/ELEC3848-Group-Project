#include <Arduino.h>
#include <Servo.h>
#include <string.h>

// Motor pin mapping reused from required_function_c1 reference sketches.
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

const uint8_t ULTRASONIC_RIGHT_TRIG_PIN = 6;
const uint8_t ULTRASONIC_RIGHT_ECHO_PIN = 4;
const uint8_t ULTRASONIC_LEFT_TRIG_PIN = 10;
const uint8_t ULTRASONIC_LEFT_ECHO_PIN = 7;
const uint8_t ARM_GRIP_SERVO_PIN = 44;
const uint8_t ARM_LIFT_SERVO_PIN = 45;

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long DISTANCE_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long COMMAND_TIMEOUT_MS = 700UL;
const unsigned long ARM_ACTIVE_WINDOW_MS = 1200UL;
const unsigned long REMOTE_UNLOCK_OPEN_MS = 5000UL;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

const char *TARGET_STATION = "MANUAL";
const int DEFAULT_MOTOR_PWM = 96;
const int DEFAULT_ARM_GRIP_DEG = 90;
const int DEFAULT_ARM_LIFT_DEG = 110;
const int MIN_MOTOR_PWM = 0;
const int MAX_MOTOR_PWM = 255;
const int MIN_ARM_DEG = 0;
const int MAX_ARM_DEG = 180;
const float LEFT_SIDE_TRIM = 1.10f;

enum DriveDirection {
  DRIVE_STOP,
  DRIVE_FORWARD,
  DRIVE_BACKWARD,
  DRIVE_LEFT,
  DRIVE_RIGHT
};

struct DistanceSnapshot {
  float rightCm;
  float leftCm;
};

unsigned long lastTelemetryMs = 0;
unsigned long lastDistanceSampleMs = 0;
unsigned long lastDriveCommandMs = 0;
unsigned long lastArmCommandMs = 0;
unsigned long remoteUnlockUntilMs = 0;

DriveDirection currentDirection = DRIVE_STOP;
bool commandTimedOut = false;
int motorPwm = DEFAULT_MOTOR_PWM;
int currentArmGripDeg = DEFAULT_ARM_GRIP_DEG;
int currentArmLiftDeg = DEFAULT_ARM_LIFT_DEG;

DistanceSnapshot latestDistances = {-1.0f, -1.0f};
float latestAverageCm = -1.0f;
float latestDiffCm = -1.0f;

Servo armGripServo;
Servo armLiftServo;

char commandBuffer[48];
uint8_t commandLength = 0;
char lastFault[40] = "";

int applyLeftTrim(int pwm) {
  return constrain(static_cast<int>(pwm * LEFT_SIDE_TRIM), MIN_MOTOR_PWM, MAX_MOTOR_PWM);
}

void setFault(const char *message) {
  strncpy(lastFault, message, sizeof(lastFault) - 1);
  lastFault[sizeof(lastFault) - 1] = '\0';
}

void clearFault() {
  lastFault[0] = '\0';
}

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

void moveForward() {
  int leftPwm = applyLeftTrim(motorPwm);
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(motorPwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(motorPwm);
}

void moveBackward() {
  int leftPwm = applyLeftTrim(motorPwm);
  MOTORA_BACKOFF(leftPwm);
  MOTORB_FORWARD(motorPwm);
  MOTORC_BACKOFF(leftPwm);
  MOTORD_FORWARD(motorPwm);
}

void strafeLeft() {
  MOTORA_BACKOFF(motorPwm);
  MOTORB_BACKOFF(motorPwm);
  MOTORC_FORWARD(motorPwm);
  MOTORD_FORWARD(motorPwm);
}

void strafeRight() {
  MOTORA_FORWARD(motorPwm);
  MOTORB_FORWARD(motorPwm);
  MOTORC_BACKOFF(motorPwm);
  MOTORD_BACKOFF(motorPwm);
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

void setupArm() {
  armGripServo.attach(ARM_GRIP_SERVO_PIN);
  armLiftServo.attach(ARM_LIFT_SERVO_PIN);
  armGripServo.write(currentArmGripDeg);
  armLiftServo.write(currentArmLiftDeg);
}

void driveCurrentDirection() {
  switch (currentDirection) {
    case DRIVE_FORWARD:
      moveForward();
      break;
    case DRIVE_BACKWARD:
      moveBackward();
      break;
    case DRIVE_LEFT:
      strafeLeft();
      break;
    case DRIVE_RIGHT:
      strafeRight();
      break;
    case DRIVE_STOP:
    default:
      stopCar();
      break;
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
  motorPwm = constrain(requestedPwm, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  if (direction != DRIVE_STOP && motorPwm == 0) {
    direction = DRIVE_STOP;
  }
  currentDirection = direction;
  lastDriveCommandMs = millis();
  commandTimedOut = false;
  clearFault();
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
  latestDistances.rightCm = readDistanceCm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
  delay(4);
  latestDistances.leftCm = readDistanceCm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);

  if (latestDistances.rightCm >= 0.0f && latestDistances.leftCm >= 0.0f) {
    latestAverageCm = (latestDistances.rightCm + latestDistances.leftCm) * 0.5f;
    latestDiffCm = latestDistances.rightCm - latestDistances.leftCm;
  } else {
    latestAverageCm = -1.0f;
    latestDiffCm = -1.0f;
  }
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

  Serial.print("\"heading_deg\":0.0,");

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
  Serial.println("Manual telemetry sketch ready.");
  Serial.println("Commands:");
  Serial.println("  DRIVE FORWARD <0-255>");
  Serial.println("  DRIVE BACKWARD <0-255>");
  Serial.println("  DRIVE LEFT <0-255>");
  Serial.println("  DRIVE RIGHT <0-255>");
  Serial.println("  DRIVE STOP <0-255>");
  Serial.println("  ARM LIFT <0-180>");
  Serial.println("  ARM GRIP <0-180>");
  Serial.println("  OPEN_DOOR");
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupUltrasonics();
  setupArm();
  stopCar();
  delay(1500);
  printHelp();
}

void loop() {
  pollSerialCommands();
  updateDriveFailsafe();
  updateDistanceSnapshot();
  driveCurrentDirection();
  emitTelemetry();
}
