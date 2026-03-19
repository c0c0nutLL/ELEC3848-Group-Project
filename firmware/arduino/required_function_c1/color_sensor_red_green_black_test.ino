#include <Arduino.h>

// Assumes the GY-31 module is based on TCS3200/TCS230.
// Update these pins to match your actual wiring.
const uint8_t COLOR_S0_PIN = 24;
const uint8_t COLOR_S1_PIN = 25;
const uint8_t COLOR_S2_PIN = 26;
const uint8_t COLOR_S3_PIN = 27;
const uint8_t COLOR_OUT_PIN = 28;
const uint8_t COLOR_LED_PIN = 29;  // Optional onboard LED control. Set LOW/HIGH as needed.

// Classification thresholds. These usually need one round of real calibration.
const uint16_t BLACK_TOTAL_THRESHOLD = 260;
const float RED_DOMINANCE_RATIO = 1.20f;
const float GREEN_DOMINANCE_RATIO = 1.20f;

struct ColorReading {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
};

void setupColorSensor() {
  pinMode(COLOR_S0_PIN, OUTPUT);
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  pinMode(COLOR_LED_PIN, OUTPUT);

  // TCS3200 output frequency scaling: 20%
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);

  // Turn on sensor illumination if the module exposes LED control.
  digitalWrite(COLOR_LED_PIN, HIGH);
}

uint16_t readColorChannel(bool s2, bool s3) {
  digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
  delay(3);

  unsigned long pulse = pulseIn(COLOR_OUT_PIN, LOW, 50000UL);
  if (pulse == 0) {
    return 0;
  }

  // TCS3200 outputs frequency. Lower pulse width means stronger reflected color.
  return static_cast<uint16_t>(pulse);
}

ColorReading readColor() {
  ColorReading reading;

  // Filter selection for common TCS3200 modules:
  // red:   S2 LOW,  S3 LOW
  // blue:  S2 LOW,  S3 HIGH
  // clear: S2 HIGH, S3 LOW
  // green: S2 HIGH, S3 HIGH
  reading.red = readColorChannel(false, false);
  reading.blue = readColorChannel(false, true);
  reading.green = readColorChannel(true, true);
  return reading;
}

const char *classifyColor(const ColorReading &reading) {
  if (reading.red == 0 || reading.green == 0 || reading.blue == 0) {
    return "NO_SIGNAL";
  }

  uint32_t total = static_cast<uint32_t>(reading.red) +
                   static_cast<uint32_t>(reading.green) +
                   static_cast<uint32_t>(reading.blue);

  // With pulse-width readings, dark surfaces usually produce larger values.
  if (total > BLACK_TOTAL_THRESHOLD) {
    return "BLACK";
  }

  // Smaller pulse width means stronger reflected component.
  float redScore = 1.0f / reading.red;
  float greenScore = 1.0f / reading.green;
  float blueScore = 1.0f / reading.blue;

  if (redScore > greenScore * RED_DOMINANCE_RATIO &&
      redScore > blueScore * RED_DOMINANCE_RATIO) {
    return "RED";
  }

  if (greenScore > redScore * GREEN_DOMINANCE_RATIO &&
      greenScore > blueScore * GREEN_DOMINANCE_RATIO) {
    return "GREEN";
  }

  return "UNKNOWN";
}

void printHelp() {
  Serial.println("Color sensor test ready.");
  Serial.println("Commands:");
  Serial.println("  R - read one sample");
  Serial.println("  C - continuous output");
  Serial.println("  S - stop continuous output");
}

bool continuousMode = true;
unsigned long lastPrintMs = 0;

void printSample() {
  ColorReading reading = readColor();
  const char *label = classifyColor(reading);

  Serial.print("R=");
  Serial.print(reading.red);
  Serial.print(" G=");
  Serial.print(reading.green);
  Serial.print(" B=");
  Serial.print(reading.blue);
  Serial.print(" -> ");
  Serial.println(label);
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'R':
    case 'r':
      printSample();
      break;
    case 'C':
    case 'c':
      continuousMode = true;
      Serial.println("Continuous mode enabled");
      break;
    case 'S':
    case 's':
      continuousMode = false;
      Serial.println("Continuous mode disabled");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupColorSensor();
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  if (continuousMode && millis() - lastPrintMs >= 250) {
    printSample();
    lastPrintMs = millis();
  }
}
