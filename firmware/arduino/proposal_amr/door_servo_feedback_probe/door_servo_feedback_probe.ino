#include <Arduino.h>
#include <Servo.h>

// Replace this with the real cargo-door servo pin after hardware confirmation.
const uint8_t TEST_SERVO_PIN = 9;

// Set to true only if the servo model exposes a readable external feedback wire.
const bool SERVO_HAS_FEEDBACK_PIN = false;
const uint8_t FEEDBACK_PIN = A0;

const int CLOSED_DEG = 20;
const int OPEN_DEG = 120;
const int CENTER_DEG = 70;
const unsigned long STEP_WAIT_MS = 900UL;

Servo testServo;

void printFeedbackSample() {
  if (!SERVO_HAS_FEEDBACK_PIN) {
    Serial.println(F("No readable external encoder/feedback pin is configured."));
    Serial.println(F("This is normal for a standard 3-wire hobby servo."));
    return;
  }

  int raw = analogRead(FEEDBACK_PIN);
  Serial.print(F("Feedback raw value: "));
  Serial.println(raw);
}

void moveAndReport(int angle) {
  testServo.write(constrain(angle, 0, 180));
  delay(STEP_WAIT_MS);

  Serial.print(F("Commanded angle: "));
  Serial.println(angle);
  printFeedbackSample();
}

void runSweep() {
  Serial.println(F("Sweep start."));
  moveAndReport(CLOSED_DEG);
  moveAndReport(CENTER_DEG);
  moveAndReport(OPEN_DEG);
  moveAndReport(CENTER_DEG);
  Serial.println(F("Sweep end."));
}

void printHelp() {
  Serial.println(F("Door servo feedback probe ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  SWEEP  - run closed/center/open sweep"));
  Serial.println(F("  OPEN   - move to open angle"));
  Serial.println(F("  CLOSE  - move to closed angle"));
  Serial.println(F("  CENTER - move to center angle"));
  Serial.println(F("  READ   - print optional feedback sample"));
  Serial.println(F("Most standard hobby servos do not expose a readable external encoder."));
}

void setup() {
  Serial.begin(115200);
  testServo.attach(TEST_SERVO_PIN);

  if (SERVO_HAS_FEEDBACK_PIN) {
    pinMode(FEEDBACK_PIN, INPUT);
  }

  testServo.write(CENTER_DEG);
  delay(STEP_WAIT_MS);
  printHelp();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "SWEEP") {
    runSweep();
  } else if (command == "OPEN") {
    moveAndReport(OPEN_DEG);
  } else if (command == "CLOSE") {
    moveAndReport(CLOSED_DEG);
  } else if (command == "CENTER") {
    moveAndReport(CENTER_DEG);
  } else if (command == "READ") {
    printFeedbackSample();
  }
}
