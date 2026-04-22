#include <Arduino.h>

enum MissionStage {
  STAGE_START = 0,
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
  STAGE_COUNT
};

MissionStage currentStage = STAGE_START;

const char *stageName(MissionStage stage) {
  switch (stage) {
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
    case STAGE_COUNT:
    default:
      return "UNKNOWN";
  }
}

void printCurrentStage() {
  Serial.print(F("Current stage: "));
  Serial.println(stageName(currentStage));
}

void printHelp() {
  Serial.println(F("Mission state machine sandbox ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  N - next stage"));
  Serial.println(F("  B - previous stage"));
  Serial.println(F("  R - reset to START"));
  Serial.println(F("  P - print current stage"));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  printHelp();
  printCurrentStage();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  char ch = static_cast<char>(Serial.read());
  switch (ch) {
    case 'N':
    case 'n':
      if (currentStage + 1 < STAGE_COUNT) {
        currentStage = static_cast<MissionStage>(currentStage + 1);
      }
      printCurrentStage();
      break;

    case 'B':
    case 'b':
      if (currentStage > STAGE_START) {
        currentStage = static_cast<MissionStage>(currentStage - 1);
      }
      printCurrentStage();
      break;

    case 'R':
    case 'r':
      currentStage = STAGE_START;
      printCurrentStage();
      break;

    case 'P':
    case 'p':
      printCurrentStage();
      break;

    default:
      break;
  }
}
