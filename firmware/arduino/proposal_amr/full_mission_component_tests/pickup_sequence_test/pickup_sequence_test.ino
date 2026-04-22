#include <Arduino.h>
#include <Servo.h>

const uint8_t ARM_GRIP_SERVO_PIN = 44;
const uint8_t ARM_LIFT_SERVO_PIN = 45;

const int HOME_LIFT_DEG = 110;
const int HOME_GRIP_DEG = 90;
const int PICK_LIFT_DEG = 72;
const int OPEN_GRIP_DEG = 120;
const int CLOSE_GRIP_DEG = 60;
const unsigned long STEP_WAIT_MS = 600UL;

Servo gripServo;
Servo liftServo;

void applyPose(int liftDeg, int gripDeg) {
  liftServo.write(constrain(liftDeg, 0, 180));
  gripServo.write(constrain(gripDeg, 0, 180));
}

void runPickupSequence() {
  Serial.println(F("Pickup sequence start."));

  applyPose(HOME_LIFT_DEG, HOME_GRIP_DEG);
  delay(STEP_WAIT_MS);

  applyPose(HOME_LIFT_DEG, OPEN_GRIP_DEG);
  delay(STEP_WAIT_MS);

  applyPose(PICK_LIFT_DEG, OPEN_GRIP_DEG);
  delay(STEP_WAIT_MS);

  applyPose(PICK_LIFT_DEG, CLOSE_GRIP_DEG);
  delay(STEP_WAIT_MS);

  applyPose(HOME_LIFT_DEG, CLOSE_GRIP_DEG);
  delay(STEP_WAIT_MS);

  applyPose(HOME_LIFT_DEG, HOME_GRIP_DEG);
  Serial.println(F("Pickup sequence end."));
}

void printHelp() {
  Serial.println(F("Pickup sequence test ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  RUN  - run pickup sequence"));
  Serial.println(F("  HOME - home pose"));
  Serial.println(F("  OPEN - open gripper"));
  Serial.println(F("  CLOSE - close gripper"));
  Serial.println(F("  UP   - lift home pose"));
  Serial.println(F("  DOWN - lower pickup pose"));
  Serial.println(F("Door-servo logic is intentionally excluded here because the door uses a separate servo and its pin is still TBD."));
}

void setup() {
  Serial.begin(115200);
  gripServo.attach(ARM_GRIP_SERVO_PIN);
  liftServo.attach(ARM_LIFT_SERVO_PIN);
  applyPose(HOME_LIFT_DEG, HOME_GRIP_DEG);
  printHelp();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "RUN") {
    runPickupSequence();
  } else if (command == "HOME") {
    applyPose(HOME_LIFT_DEG, HOME_GRIP_DEG);
    Serial.println(F("Home pose."));
  } else if (command == "OPEN") {
    applyPose(HOME_LIFT_DEG, OPEN_GRIP_DEG);
    Serial.println(F("Gripper open."));
  } else if (command == "CLOSE") {
    applyPose(HOME_LIFT_DEG, CLOSE_GRIP_DEG);
    Serial.println(F("Gripper close."));
  } else if (command == "UP") {
    applyPose(HOME_LIFT_DEG, HOME_GRIP_DEG);
    Serial.println(F("Lift up."));
  } else if (command == "DOWN") {
    applyPose(PICK_LIFT_DEG, HOME_GRIP_DEG);
    Serial.println(F("Lift down."));
  }
}
