#include <Arduino.h>

const unsigned long TELEMETRY_INTERVAL_MS = 200UL;
const unsigned long DEMO_CYCLE_MS = 30000UL;
const unsigned long REMOTE_UNLOCK_OPEN_MS = 5000UL;

const char *TARGET_STATION = "A1";

unsigned long lastTelemetryMs = 0;
unsigned long remoteUnlockUntilMs = 0;

char commandBuffer[32];
uint8_t commandLength = 0;

float clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

float lerp(float startValue, float endValue, float t) {
  return startValue + (endValue - startValue) * clamp01(t);
}

void printJsonString(const char *value) {
  Serial.print('"');
  Serial.print(value);
  Serial.print('"');
}

void handleCommand(const char *command) {
  if (strcmp(command, "OPEN_DOOR") == 0) {
    remoteUnlockUntilMs = millis() + REMOTE_UNLOCK_OPEN_MS;
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

void emitTelemetry(
  const char *state,
  float rightCm,
  float leftCm,
  float headingDeg,
  const char *rfidTag,
  bool rfidMatch,
  const char *cargoDoor,
  const char *deliveryResult,
  const char *fault
) {
  float averageCm = (rightCm + leftCm) * 0.5f;
  float diffCm = rightCm - leftCm;

  Serial.print('{');

  Serial.print("\"ts_ms\":");
  Serial.print(millis());
  Serial.print(',');

  Serial.print("\"state\":");
  printJsonString(state);
  Serial.print(',');

  Serial.print("\"target_station\":");
  printJsonString(TARGET_STATION);
  Serial.print(',');

  Serial.print("\"right_cm\":");
  Serial.print(rightCm, 1);
  Serial.print(',');

  Serial.print("\"left_cm\":");
  Serial.print(leftCm, 1);
  Serial.print(',');

  Serial.print("\"avg_cm\":");
  Serial.print(averageCm, 1);
  Serial.print(',');

  Serial.print("\"diff_cm\":");
  Serial.print(diffCm, 1);
  Serial.print(',');

  Serial.print("\"heading_deg\":");
  Serial.print(headingDeg, 1);
  Serial.print(',');

  Serial.print("\"rfid_tag\":");
  printJsonString(rfidTag);
  Serial.print(',');

  Serial.print("\"rfid_match\":");
  Serial.print(rfidMatch ? "true" : "false");
  Serial.print(',');

  Serial.print("\"cargo_door\":");
  printJsonString(cargoDoor);
  Serial.print(',');

  Serial.print("\"delivery_result\":");
  printJsonString(deliveryResult);
  Serial.print(',');

  Serial.print("\"fault\":");
  printJsonString(fault);

  Serial.println('}');
}

void setup() {
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  pollSerialCommands();

  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = millis();

  unsigned long cycleMs = millis() % DEMO_CYCLE_MS;

  const char *state = "IDLE";
  float rightCm = 100.0f;
  float leftCm = 100.0f;
  float headingDeg = 0.0f;
  const char *rfidTag = "";
  bool rfidMatch = false;
  const char *cargoDoor = "CLOSED";
  const char *deliveryResult = "READY";
  const char *fault = "";

  if (cycleMs < 4000UL) {
    state = "IDLE";
    deliveryResult = "READY";
  } else if (cycleMs < 12000UL) {
    float t = (cycleMs - 4000UL) / 8000.0f;
    state = "MOVING";
    rightCm = lerp(95.0f, 45.0f, t);
    leftCm = lerp(92.0f, 43.0f, t);
    headingDeg = lerp(0.0f, 2.5f, t);
    deliveryResult = "RUNNING";
  } else if (cycleMs < 17000UL) {
    float t = (cycleMs - 12000UL) / 5000.0f;
    state = "DOCKING";
    rightCm = lerp(42.0f, 12.0f, t);
    leftCm = lerp(38.0f, 12.5f, t);
    headingDeg = lerp(2.5f, 0.6f, t);
    deliveryResult = "RUNNING";
  } else if (cycleMs < 20000UL) {
    state = "RFID_CHECK";
    rightCm = 12.0f;
    leftCm = 12.2f;
    headingDeg = 0.4f;
    rfidTag = "STATION_A1";
    rfidMatch = true;
    deliveryResult = "VERIFYING";
  } else if (cycleMs < 23000UL) {
    state = "CARGO_RELEASE";
    rightCm = 12.0f;
    leftCm = 12.1f;
    headingDeg = 0.3f;
    rfidTag = "STATION_A1";
    rfidMatch = true;
    cargoDoor = "OPEN";
    deliveryResult = "DELIVERING";
  } else if (cycleMs < 26000UL) {
    state = "REPORTING";
    rightCm = 12.0f;
    leftCm = 12.0f;
    headingDeg = 0.2f;
    rfidTag = "STATION_A1";
    rfidMatch = true;
    cargoDoor = "CLOSED";
    deliveryResult = "UPLOADING";
  } else {
    state = "DONE";
    rightCm = 12.0f;
    leftCm = 12.0f;
    headingDeg = 0.2f;
    rfidTag = "STATION_A1";
    rfidMatch = true;
    cargoDoor = "CLOSED";
    deliveryResult = "SUCCESS";
  }

  if (remoteUnlockUntilMs > millis()) {
    state = "REMOTE_UNLOCK";
    cargoDoor = "OPEN";
    deliveryResult = "UNLOCKED";
  }

  emitTelemetry(
    state,
    rightCm,
    leftCm,
    headingDeg,
    rfidTag,
    rfidMatch,
    cargoDoor,
    deliveryResult,
    fault
  );
}
