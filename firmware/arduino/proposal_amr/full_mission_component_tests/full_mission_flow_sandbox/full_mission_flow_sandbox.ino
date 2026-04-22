#include <Arduino.h>
#include <Servo.h>
#include <string.h>

// Chassis pin map reused from the current proposal baseline.
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

const uint8_t FRONT_RIGHT_TRIG_PIN = 6;   // ASSUMED
const uint8_t FRONT_RIGHT_ECHO_PIN = 4;   // ASSUMED
const uint8_t FRONT_LEFT_TRIG_PIN = 10;   // ASSUMED
const uint8_t FRONT_LEFT_ECHO_PIN = 7;    // ASSUMED
const uint8_t ARM_GRIP_SERVO_PIN = 44;    // ASSUMED
const uint8_t ARM_LIFT_SERVO_PIN = 45;    // ASSUMED
const uint8_t DOOR_SERVO_PIN = 46;        // ASSUMED
const uint8_t RFID_SS_PIN = 53;           // ASSUMED
const uint8_t RFID_RST_PIN = 49;          // ASSUMED
const uint8_t LOCAL_START_BUTTON_PIN = 33;  // ASSUMED
const uint8_t LOCAL_CLOSE_BUTTON_PIN = 32;  // ASSUMED

const char PICKUP_RFID_TAG[] = "PICKUP_TAG_01";      // ASSUMED
const char DELIVERY_RFID_TAG[] = "DELIVERY_TAG_01";  // ASSUMED

const int DEFAULT_TRAVEL_PWM = 92;      // ASSUMED
const int DEFAULT_STRAFE_PWM = 88;      // ASSUMED
const int DEFAULT_TURN_PWM = 96;        // ASSUMED
const int DOOR_CLOSED_DEG = 18;         // ASSUMED
const int DOOR_OPEN_DEG = 108;          // ASSUMED
const int ARM_HOME_LIFT_DEG = 110;      // ASSUMED
const int ARM_HOME_GRIP_DEG = 90;       // ASSUMED
const int ARM_PICK_LIFT_DEG = 72;       // ASSUMED
const int ARM_OPEN_GRIP_DEG = 120;      // ASSUMED
const int ARM_CLOSE_GRIP_DEG = 60;      // ASSUMED

const unsigned long START_SETTLE_MS = 500UL;              // ASSUMED
const unsigned long PICKUP_STEP_WAIT_MS = 600UL;          // ASSUMED
const unsigned long LOAD_SETTLE_MS = 2500UL;              // ASSUMED
const unsigned long DOOR_LOAD_SETTLE_MS = 1200UL;         // ASSUMED
const unsigned long TURN_AROUND_MS = 720UL;               // ASSUMED
const unsigned long DELIVERY_DOOR_AUTO_CLOSE_MS = 60000UL;  // ASSUMED
const unsigned long STATUS_INTERVAL_MS = 1000UL;          // ASSUMED
const unsigned long OBSTACLE_STOP_HOLD_MS = 400UL;        // ASSUMED
const unsigned long OBSTACLE_STRAFE_MS = 850UL;           // ASSUMED
const unsigned long OBSTACLE_PASS_MS = 1300UL;            // ASSUMED
const unsigned long OBSTACLE_RETURN_MS = 850UL;           // ASSUMED

const float OBSTACLE_TRIGGER_CM = 45.0f;      // ASSUMED
const float OBSTACLE_CLEAR_CM = 65.0f;        // ASSUMED
const float DEFAULT_SIM_FRONT_CM = 120.0f;    // ASSUMED
const float DEFAULT_OBSTACLE_TEST_CM = 30.0f; // ASSUMED
const float IDLE_FRONT_MARGIN_CM = 20.0f;     // ASSUMED
const float CLEAR_FRONT_MARGIN_CM = 10.0f;    // ASSUMED
const float FORWARD_LEFT_TRIM = 1.08f;        // ASSUMED
const float STRAFE_LEFT_RF_TRIM = 1.05f;      // ASSUMED
const float STRAFE_RIGHT_RF_TRIM = 1.10f;     // ASSUMED
const float STRAFE_RIGHT_LR_TRIM = 1.10f;     // ASSUMED

enum MissionTask {
  TASK_NONE = 0,
  TASK_FULL_MISSION,
  TASK_PICKUP_MISSION,
  TASK_DELIVERY_MISSION,
  TASK_OBSTACLE_MISSION,
  TASK_MANUAL_TEST
};

enum MissionStage {
  STAGE_IDLE = 0,
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
  STAGE_DONE
};

enum ObstacleKind {
  OBSTACLE_NONE = 0,
  OBSTACLE_PERSON,
  OBSTACLE_OBJECT
};

Servo gripServo;
Servo liftServo;
Servo doorServo;

MissionTask currentTask = TASK_NONE;
MissionStage currentStage = STAGE_IDLE;
ObstacleKind obstacleKind = OBSTACLE_NONE;

bool missionActive = false;
bool stageEntryHandled = false;
bool pickupRfidMatched = false;
bool deliveryRfidMatched = false;
bool unlockApproved = false;
bool doorOpen = false;
bool doorManualCloseRequested = false;
bool doorAutoClosed = false;
bool obstacleSeen = false;
bool obstacleRoutineActive = false;

uint8_t obstacleRoutineStep = 0;
unsigned long stageStartedMs = 0;
unsigned long deliveryDoorOpenedMs = 0;
unsigned long obstacleStepStartedMs = 0;
unsigned long lastStatusMs = 0;

float simulatedFrontDistanceCm = DEFAULT_SIM_FRONT_CM;
char lastSeenRfid[24] = "";

const char *taskName(MissionTask task) {
  switch (task) {
    case TASK_FULL_MISSION:
      return "FULL_MISSION";
    case TASK_PICKUP_MISSION:
      return "PICKUP_MISSION";
    case TASK_DELIVERY_MISSION:
      return "DELIVERY_MISSION";
    case TASK_OBSTACLE_MISSION:
      return "OBSTACLE_MISSION";
    case TASK_MANUAL_TEST:
      return "MANUAL_TEST";
    case TASK_NONE:
    default:
      return "NONE";
  }
}

const char *stageName(MissionStage stage) {
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
    default:
      return "UNKNOWN";
  }
}

const char *obstacleName(ObstacleKind kind) {
  switch (kind) {
    case OBSTACLE_PERSON:
      return "PERSON";
    case OBSTACLE_OBJECT:
      return "OBJECT";
    case OBSTACLE_NONE:
    default:
      return "NONE";
  }
}

int trimPwm(int pwm, float factor) {
  int scaled = static_cast<int>(pwm * factor + 0.5f);
  return constrain(scaled, 0, 255);
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

void setupInputPins() {
  pinMode(LOCAL_START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LOCAL_CLOSE_BUTTON_PIN, INPUT_PULLUP);
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

void rotateRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

void applyArmPose(int liftDeg, int gripDeg) {
  liftServo.write(constrain(liftDeg, 0, 180));
  gripServo.write(constrain(gripDeg, 0, 180));
}

void openDoor() {
  doorServo.write(constrain(DOOR_OPEN_DEG, 0, 180));
  doorOpen = true;
}

void closeDoor() {
  doorServo.write(constrain(DOOR_CLOSED_DEG, 0, 180));
  doorOpen = false;
  deliveryDoorOpenedMs = 0;
}

void emitStatus() {
  Serial.print(F("{\"task\":\""));
  Serial.print(taskName(currentTask));
  Serial.print(F("\",\"stage\":\""));
  Serial.print(stageName(currentStage));
  Serial.print(F("\",\"mission_active\":"));
  Serial.print(missionActive ? F("true") : F("false"));
  Serial.print(F(",\"cargo_door\":\""));
  Serial.print(doorOpen ? F("OPEN") : F("CLOSED"));
  Serial.print(F("\",\"rfid\":\""));
  Serial.print(lastSeenRfid);
  Serial.print(F("\",\"front_cm\":"));
  Serial.print(simulatedFrontDistanceCm, 1);
  Serial.print(F(",\"obstacle\":\""));
  Serial.print(obstacleName(obstacleKind));
  Serial.print(F("\",\"auto_closed\":"));
  Serial.print(doorAutoClosed ? F("true") : F("false"));
  Serial.println(F("}"));
}

void setStage(MissionStage nextStage) {
  currentStage = nextStage;
  stageStartedMs = millis();
  stageEntryHandled = false;
  Serial.print(F("STAGE -> "));
  Serial.println(stageName(currentStage));
}

void clearMissionFlags() {
  pickupRfidMatched = false;
  deliveryRfidMatched = false;
  unlockApproved = false;
  doorManualCloseRequested = false;
  doorAutoClosed = false;
  obstacleSeen = false;
  obstacleKind = OBSTACLE_NONE;
  obstacleRoutineActive = false;
  obstacleRoutineStep = 0;
  simulatedFrontDistanceCm = OBSTACLE_CLEAR_CM + IDLE_FRONT_MARGIN_CM;
  lastSeenRfid[0] = '\0';
}

void beginMission(MissionTask task) {
  stopCar();
  clearMissionFlags();
  closeDoor();
  applyArmPose(ARM_HOME_LIFT_DEG, ARM_HOME_GRIP_DEG);
  currentTask = task;
  missionActive = true;
  Serial.print(F("MISSION -> "));
  Serial.println(taskName(currentTask));
  setStage(STAGE_START);
}

void abortMission() {
  stopCar();
  closeDoor();
  missionActive = false;
  currentTask = TASK_NONE;
  clearMissionFlags();
  setStage(STAGE_IDLE);
  Serial.println(F("Mission aborted/reset."));
}

void runPickupSequence() {
  Serial.println(F("Pickup sequence start."));
  applyArmPose(ARM_HOME_LIFT_DEG, ARM_HOME_GRIP_DEG);
  delay(PICKUP_STEP_WAIT_MS);
  applyArmPose(ARM_HOME_LIFT_DEG, ARM_OPEN_GRIP_DEG);
  delay(PICKUP_STEP_WAIT_MS);
  applyArmPose(ARM_PICK_LIFT_DEG, ARM_OPEN_GRIP_DEG);
  delay(PICKUP_STEP_WAIT_MS);
  applyArmPose(ARM_PICK_LIFT_DEG, ARM_CLOSE_GRIP_DEG);
  delay(PICKUP_STEP_WAIT_MS);
  applyArmPose(ARM_HOME_LIFT_DEG, ARM_CLOSE_GRIP_DEG);
  delay(PICKUP_STEP_WAIT_MS);
  applyArmPose(ARM_HOME_LIFT_DEG, ARM_HOME_GRIP_DEG);
  Serial.println(F("Pickup sequence end."));
}

bool stageSupportsObstacle(MissionStage stage) {
  return stage == STAGE_TRAVEL_TO_PICKUP || stage == STAGE_TRAVEL_TO_DELIVERY;
}

bool shouldStartObstacleRoutine() {
  return missionActive &&
         stageSupportsObstacle(currentStage) &&
         obstacleSeen &&
         simulatedFrontDistanceCm > 0.0f &&
         simulatedFrontDistanceCm < OBSTACLE_TRIGGER_CM &&
         !obstacleRoutineActive;
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
  Serial.print(F("Obstacle interrupt start: "));
  Serial.print(obstacleName(obstacleKind));
  Serial.print(F(" at "));
  Serial.print(simulatedFrontDistanceCm, 1);
  Serial.println(F(" cm."));
}

void updateObstacleRoutine() {
  unsigned long elapsedMs = millis() - obstacleStepStartedMs;

  switch (obstacleRoutineStep) {
    case 0:
      stopCar();
      if (elapsedMs >= OBSTACLE_STOP_HOLD_MS) {
        advanceObstacleRoutine();
      }
      break;

    case 1:
      strafeRight(DEFAULT_STRAFE_PWM);
      if (elapsedMs >= OBSTACLE_STRAFE_MS) {
        advanceObstacleRoutine();
      }
      break;

    case 2:
      moveForward(DEFAULT_TRAVEL_PWM);
      if (elapsedMs >= OBSTACLE_PASS_MS) {
        advanceObstacleRoutine();
      }
      break;

    case 3:
      strafeLeft(DEFAULT_STRAFE_PWM);
      if (elapsedMs >= OBSTACLE_RETURN_MS) {
        advanceObstacleRoutine();
      }
      break;

    default:
      stopCar();
      obstacleRoutineActive = false;
      obstacleSeen = false;
      obstacleKind = OBSTACLE_NONE;
      simulatedFrontDistanceCm = OBSTACLE_CLEAR_CM + CLEAR_FRONT_MARGIN_CM;
      Serial.println(F("Obstacle interrupt cleared. Resume mission stage."));
      break;
  }
}

void injectRfid(const char *tagText, bool pickupMatch, bool deliveryMatch) {
  strncpy(lastSeenRfid, tagText, sizeof(lastSeenRfid) - 1);
  lastSeenRfid[sizeof(lastSeenRfid) - 1] = '\0';
  pickupRfidMatched = pickupMatch;
  deliveryRfidMatched = deliveryMatch;
  Serial.print(F("RFID -> "));
  Serial.println(lastSeenRfid);
}

void clearObstacleInput() {
  obstacleSeen = false;
  obstacleKind = OBSTACLE_NONE;
  simulatedFrontDistanceCm = OBSTACLE_CLEAR_CM + CLEAR_FRONT_MARGIN_CM;
  Serial.println(F("Obstacle input cleared."));
}

void injectObstacle(ObstacleKind kind, float distanceCm) {
  obstacleSeen = true;
  obstacleKind = kind;
  if (distanceCm > 0.0f) {
    simulatedFrontDistanceCm = distanceCm;
  }
  Serial.print(F("Obstacle input -> "));
  Serial.print(obstacleName(kind));
  Serial.print(F(" at "));
  Serial.print(simulatedFrontDistanceCm, 1);
  Serial.println(F(" cm."));
}

float parseTrailingNumber(const String &commandText, const String &prefixText) {
  if (!commandText.startsWith(prefixText)) {
    return -1.0f;
  }

  String trailing = commandText.substring(prefixText.length());
  trailing.trim();
  if (trailing.length() == 0) {
    return -1.0f;
  }
  return trailing.toFloat();
}

void printHelp() {
  Serial.println(F("Full mission flow sandbox ready."));
  Serial.println(F("Task starts:"));
  Serial.println(F("  START FULL"));
  Serial.println(F("  START PICKUP"));
  Serial.println(F("  START DELIVERY"));
  Serial.println(F("Mission controls:"));
  Serial.println(F("  ABORT"));
  Serial.println(F("  RESET"));
  Serial.println(F("  STATUS"));
  Serial.println(F("RFID simulation:"));
  Serial.println(F("  RFID PICKUP"));
  Serial.println(F("  RFID DELIVERY"));
  Serial.println(F("  RFID CLEAR"));
  Serial.println(F("Unlock / door:"));
  Serial.println(F("  OPEN_DOOR"));
  Serial.println(F("  CLOSE_DOOR"));
  Serial.println(F("Obstacle simulation:"));
  Serial.println(F("  OBSTACLE PERSON <cm>"));
  Serial.println(F("  OBSTACLE OBJECT <cm>"));
  Serial.println(F("  OBSTACLE CLEAR"));
  Serial.println(F("Distance override:"));
  Serial.println(F("  DIST <cm>"));
  Serial.println(F("This sketch is a structure sandbox. RFID, door-servo pin, and motion timing all still use assumed values."));
}

void handleOpenDoorCommand() {
  if (currentStage == STAGE_WAIT_USER_UNLOCK) {
    unlockApproved = true;
    Serial.println(F("Unlock approved by Jetson. Proceed to door-open stage."));
    return;
  }

  openDoor();
  Serial.println(F("Door opened by direct bench command."));
}

void handleCloseDoorCommand() {
  if (currentStage == STAGE_WAIT_DOOR_CLOSE && doorOpen) {
    doorManualCloseRequested = true;
    Serial.println(F("Manual close requested during unload."));
    return;
  }

  closeDoor();
  Serial.println(F("Door closed by direct command."));
}

void handleCommand(const String &rawCommand) {
  String command = rawCommand;
  command.trim();
  if (command.length() == 0) {
    return;
  }

  String upper = command;
  upper.toUpperCase();

  if (upper == "HELP") {
    printHelp();
    return;
  }

  if (upper == "STATUS") {
    emitStatus();
    return;
  }

  if (upper == "START FULL") {
    beginMission(TASK_FULL_MISSION);
    return;
  }

  if (upper == "START PICKUP") {
    beginMission(TASK_PICKUP_MISSION);
    return;
  }

  if (upper == "START DELIVERY") {
    beginMission(TASK_DELIVERY_MISSION);
    return;
  }

  if (upper == "ABORT" || upper == "RESET") {
    abortMission();
    return;
  }

  if (upper == "RFID PICKUP") {
    injectRfid(PICKUP_RFID_TAG, true, false);
    return;
  }

  if (upper == "RFID DELIVERY") {
    injectRfid(DELIVERY_RFID_TAG, false, true);
    return;
  }

  if (upper == "RFID CLEAR") {
    injectRfid("", false, false);
    return;
  }

  if (upper == "OPEN_DOOR") {
    handleOpenDoorCommand();
    return;
  }

  if (upper == "CLOSE_DOOR") {
    handleCloseDoorCommand();
    return;
  }

  if (upper.startsWith("DIST ")) {
    float newDistance = parseTrailingNumber(upper, "DIST");
    if (newDistance > 0.0f) {
      simulatedFrontDistanceCm = newDistance;
      Serial.print(F("Front distance override -> "));
      Serial.print(simulatedFrontDistanceCm, 1);
      Serial.println(F(" cm."));
    }
    return;
  }

  if (upper.startsWith("OBSTACLE PERSON")) {
    float newDistance = parseTrailingNumber(upper, "OBSTACLE PERSON");
    injectObstacle(OBSTACLE_PERSON, newDistance > 0.0f ? newDistance : DEFAULT_OBSTACLE_TEST_CM);
    return;
  }

  if (upper.startsWith("OBSTACLE OBJECT")) {
    float newDistance = parseTrailingNumber(upper, "OBSTACLE OBJECT");
    injectObstacle(OBSTACLE_OBJECT, newDistance > 0.0f ? newDistance : DEFAULT_OBSTACLE_TEST_CM);
    return;
  }

  if (upper == "OBSTACLE CLEAR") {
    clearObstacleInput();
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(command);
}

void updateLocalButtons() {
  static uint8_t lastStartState = HIGH;
  static uint8_t lastCloseState = HIGH;

  uint8_t startState = digitalRead(LOCAL_START_BUTTON_PIN);
  if (lastStartState == HIGH && startState == LOW && !missionActive) {
    beginMission(TASK_FULL_MISSION);
  }
  lastStartState = startState;

  uint8_t closeState = digitalRead(LOCAL_CLOSE_BUTTON_PIN);
  if (lastCloseState == HIGH && closeState == LOW && doorOpen) {
    handleCloseDoorCommand();
  }
  lastCloseState = closeState;
}

void reportMissionResult() {
  Serial.print(F("MISSION RESULT -> task="));
  Serial.print(taskName(currentTask));
  Serial.print(F(", door_close="));
  Serial.print(doorAutoClosed ? F("AUTO_TIMEOUT") : F("MANUAL_OR_STAGE"));
  Serial.print(F(", last_rfid="));
  Serial.println(lastSeenRfid);
}

void updateMissionStateMachine() {
  if (!missionActive) {
    return;
  }

  if (shouldStartObstacleRoutine()) {
    startObstacleRoutine();
  }

  if (obstacleRoutineActive) {
    updateObstacleRoutine();
    return;
  }

  switch (currentStage) {
    case STAGE_START:
      stopCar();
      if (millis() - stageStartedMs >= START_SETTLE_MS) {
        if (currentTask == TASK_FULL_MISSION || currentTask == TASK_PICKUP_MISSION) {
          setStage(STAGE_TRAVEL_TO_PICKUP);
        } else if (currentTask == TASK_DELIVERY_MISSION) {
          setStage(STAGE_TRAVEL_TO_DELIVERY);
        }
      }
      break;

    case STAGE_TRAVEL_TO_PICKUP:
      moveForward(DEFAULT_TRAVEL_PWM);
      if (pickupRfidMatched) {
        stopCar();
        setStage(STAGE_VERIFY_PICKUP_STATION);
      }
      break;

    case STAGE_VERIFY_PICKUP_STATION:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        Serial.println(F("Pickup RFID verified."));
        setStage(STAGE_PICKUP_SEQUENCE);
      }
      break;

    case STAGE_PICKUP_SEQUENCE:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        runPickupSequence();
        setStage(STAGE_OPEN_DOOR_FOR_LOAD);
      }
      break;

    case STAGE_OPEN_DOOR_FOR_LOAD:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        openDoor();
        delay(DOOR_LOAD_SETTLE_MS);
        setStage(STAGE_LOAD_CARGO);
      }
      break;

    case STAGE_LOAD_CARGO:
      stopCar();
      if (millis() - stageStartedMs >= LOAD_SETTLE_MS) {
        setStage(STAGE_CLOSE_DOOR_AFTER_LOAD);
      }
      break;

    case STAGE_CLOSE_DOOR_AFTER_LOAD:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        closeDoor();
        if (currentTask == TASK_PICKUP_MISSION) {
          setStage(STAGE_REPORT_STATUS);
        } else {
          setStage(STAGE_TURN_AROUND);
        }
      }
      break;

    case STAGE_TURN_AROUND:
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        Serial.println(F("Turn-around start."));
        rotateRight(DEFAULT_TURN_PWM);
        delay(TURN_AROUND_MS);
        stopCar();
        setStage(STAGE_TRAVEL_TO_DELIVERY);
      }
      break;

    case STAGE_TRAVEL_TO_DELIVERY:
      moveForward(DEFAULT_TRAVEL_PWM);
      if (deliveryRfidMatched) {
        stopCar();
        setStage(STAGE_VERIFY_DELIVERY_STATION);
      }
      break;

    case STAGE_VERIFY_DELIVERY_STATION:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        Serial.println(F("Delivery RFID verified."));
        setStage(STAGE_WAIT_USER_UNLOCK);
      }
      break;

    case STAGE_WAIT_USER_UNLOCK:
      stopCar();
      if (unlockApproved) {
        unlockApproved = false;
        setStage(STAGE_OPEN_DOOR_FOR_UNLOAD);
      }
      break;

    case STAGE_OPEN_DOOR_FOR_UNLOAD:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        openDoor();
        deliveryDoorOpenedMs = millis();
        doorManualCloseRequested = false;
        doorAutoClosed = false;
        Serial.print(F("Unload door opened. CLOSE_DOOR closes it immediately; otherwise auto-close fires after "));
        Serial.print(DELIVERY_DOOR_AUTO_CLOSE_MS / 1000UL);
        Serial.println(F(" seconds."));
        setStage(STAGE_WAIT_DOOR_CLOSE);
      }
      break;

    case STAGE_WAIT_DOOR_CLOSE:
      stopCar();
      if (doorManualCloseRequested) {
        closeDoor();
        doorManualCloseRequested = false;
        setStage(STAGE_REPORT_STATUS);
      } else if (deliveryDoorOpenedMs > 0 &&
                 millis() - deliveryDoorOpenedMs >= DELIVERY_DOOR_AUTO_CLOSE_MS) {
        setStage(STAGE_AUTO_CLOSE_DOOR);
      }
      break;

    case STAGE_AUTO_CLOSE_DOOR:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        closeDoor();
        doorAutoClosed = true;
        Serial.println(F("Door auto-closed after timeout."));
        setStage(STAGE_REPORT_STATUS);
      }
      break;

    case STAGE_REPORT_STATUS:
      stopCar();
      if (!stageEntryHandled) {
        stageEntryHandled = true;
        reportMissionResult();
        setStage(STAGE_DONE);
      }
      break;

    case STAGE_DONE:
      stopCar();
      missionActive = false;
      break;

    case STAGE_IDLE:
    default:
      stopCar();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupInputPins();
  stopCar();

  gripServo.attach(ARM_GRIP_SERVO_PIN);
  liftServo.attach(ARM_LIFT_SERVO_PIN);
  doorServo.attach(DOOR_SERVO_PIN);

  applyArmPose(ARM_HOME_LIFT_DEG, ARM_HOME_GRIP_DEG);
  closeDoor();
  printHelp();
  emitStatus();
}

void loop() {
  updateLocalButtons();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }

  updateMissionStateMachine();

  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    emitStatus();
  }
}
