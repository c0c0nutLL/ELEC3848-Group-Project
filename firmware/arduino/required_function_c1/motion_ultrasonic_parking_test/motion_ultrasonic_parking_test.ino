#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Dual-ultrasonic parking test with real-time OLED telemetry.
// ultrasonic1 is mounted on the right side of the front bumper.
// ultrasonic2 is mounted on the left side of the front bumper.
// A larger right distance means the car is farther from the wall on the right,
// so the heading correction should turn left.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

// Motor pins and direction macros follow reference.ino.
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

// Pin mapping from amr_pin_mapping_template.xlsx.
// ultrasonic1: right side, TRIG=6, ECHO=4
// ultrasonic2: left side, TRIG=10, ECHO=7
const uint8_t ULTRASONIC1_TRIG_PIN = 6;
const uint8_t ULTRASONIC1_ECHO_PIN = 4;
const uint8_t ULTRASONIC2_TRIG_PIN = 10;
const uint8_t ULTRASONIC2_ECHO_PIN = 7;
const uint8_t START_BUTTON_PIN = 33;

// Current test target: stop at about 25 cm from the wall.
const float TARGET_DISTANCE_CM = 25.0f;
const float DISTANCE_TOLERANCE_CM = 1.0f;
const float SLOWDOWN_START_DISTANCE_CM = 45.0f;
const float ALIGN_TOLERANCE_CM = 3.5f;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

// Lower PWM values make the parking test easier to tune.
const int FAST_PWM = 72;
const int SLOW_PWM = 58;
const int TURN_PWM = 60;

bool autoParkingEnabled = false;
unsigned long lastTelemetryMs = 0;
bool lastButtonPressed = false;
unsigned long lastIdleSampleMs = 0;

struct DistancePair {
  float rightCm;
  float leftCm;
};

DistancePair lastPair = {-1.0f, -1.0f};
float lastAverageCm = -1.0f;
float lastDiffCm = 0.0f;
const char *lastAction = "READY";

void updateOled(const DistancePair &pair, float averageCm, float diffCm, const char *action) {
  if (!oledReady) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("R:");
  display.print(pair.rightCm, 1);
  display.print(" L:");
  display.println(pair.leftCm, 1);

  display.print("Avg:");
  display.print(averageCm, 1);
  display.print(" D:");
  display.println(diffCm, 1);

  display.print("Act:");
  display.println(action);
  display.display();
}

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

void moveForward(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_BACKOFF(pwm);
}

// Match the rotation definitions used in reference.ino:
// all FORWARD equals rotate_2.
void rotateRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

// All BACKOFF equals rotate_1.
void rotateLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
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
  pinMode(ULTRASONIC1_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC1_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC2_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC2_ECHO_PIN, INPUT);

  digitalWrite(ULTRASONIC1_TRIG_PIN, LOW);
  digitalWrite(ULTRASONIC2_TRIG_PIN, LOW);
}

void setupButton() {
  // The button is wired to GND, so pressed reads LOW.
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
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("AI Robot");
  display.display();
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

DistancePair readDistances() {
  DistancePair result;
  result.rightCm = readDistanceCm(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
  delay(5);
  result.leftCm = readDistanceCm(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);
  return result;
}

void saveLatestReadings(const DistancePair &pair, float averageCm, float diffCm, const char *action) {
  lastPair = pair;
  lastAverageCm = averageCm;
  lastDiffCm = diffCm;
  lastAction = action;
}

int computeForwardPwm(float averageCm) {
  if (averageCm >= SLOWDOWN_START_DISTANCE_CM) {
    return FAST_PWM;
  }

  if (averageCm <= TARGET_DISTANCE_CM) {
    return SLOW_PWM;
  }

  float ratio = (averageCm - TARGET_DISTANCE_CM) /
                (SLOWDOWN_START_DISTANCE_CM - TARGET_DISTANCE_CM);
  float pwm = SLOW_PWM + ratio * (FAST_PWM - SLOW_PWM);
  return static_cast<int>(pwm);
}

void printHelp() {
  Serial.println("Dual ultrasonic parking test ready.");
  Serial.println("Commands:");
  Serial.println("  G - start auto parking");
  Serial.println("  X - stop auto parking");
  Serial.println("  F - manual forward");
  Serial.println("  L - manual rotate left");
  Serial.println("  R - manual rotate right");
  Serial.println("  S - stop");
  Serial.println("  D - print one distance sample");
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'G':
    case 'g':
      autoParkingEnabled = true;
      Serial.println("Auto parking enabled");
      break;
    case 'X':
    case 'x':
      autoParkingEnabled = false;
      stopCar();
      Serial.println("Auto parking disabled");
      break;
    case 'F':
    case 'f':
      autoParkingEnabled = false;
      moveForward(FAST_PWM);
      Serial.println("Manual forward");
      break;
    case 'L':
    case 'l':
      autoParkingEnabled = false;
      rotateLeft(TURN_PWM);
      Serial.println("Manual rotate left");
      break;
    case 'R':
    case 'r':
      autoParkingEnabled = false;
      rotateRight(TURN_PWM);
      Serial.println("Manual rotate right");
      break;
    case 'S':
    case 's':
      autoParkingEnabled = false;
      stopCar();
      Serial.println("Stop");
      break;
    case 'D':
    case 'd': {
      DistancePair pair = readDistances();
      float averageCm = (pair.rightCm + pair.leftCm) * 0.5f;
      float diffCm = pair.rightCm - pair.leftCm;
      Serial.print("Right(cm): ");
      Serial.print(pair.rightCm);
      Serial.print(" Left(cm): ");
      Serial.println(pair.leftCm);
      saveLatestReadings(pair, averageCm, diffCm, "SAMPLE");
      updateOled(pair, averageCm, diffCm, "SAMPLE");
      break;
    }
    default:
      break;
  }
}

void runAutoParking() {
  DistancePair pair = readDistances();
  if (pair.rightCm < 0.0f || pair.leftCm < 0.0f) {
    stopCar();
    Serial.println("Ultrasonic timeout");
    saveLatestReadings(pair, -1.0f, 0.0f, "TIMEOUT");
    updateOled(pair, -1.0f, 0.0f, "TIMEOUT");
    return;
  }

  float averageCm = (pair.rightCm + pair.leftCm) * 0.5f;
  float diffCm = pair.rightCm - pair.leftCm;

  if (millis() - lastTelemetryMs >= 200) {
    Serial.print("Right(cm): ");
    Serial.print(pair.rightCm);
    Serial.print(" Left(cm): ");
    Serial.print(pair.leftCm);
    Serial.print(" Avg(cm): ");
    Serial.print(averageCm);
    Serial.print(" Diff(cm): ");
    Serial.println(diffCm);
    lastTelemetryMs = millis();
  }

  if (averageCm <= TARGET_DISTANCE_CM + DISTANCE_TOLERANCE_CM) {
    stopCar();
    autoParkingEnabled = false;
    Serial.println("Parking complete");
    saveLatestReadings(pair, averageCm, diffCm, "STOP");
    updateOled(pair, averageCm, diffCm, "STOP");
    return;
  }

  int drivePwm = computeForwardPwm(averageCm);

  // Right side is farther from the wall, so turn left to correct.
  if (diffCm > ALIGN_TOLERANCE_CM) {
    rotateLeft(TURN_PWM);
    Serial.println("Action: LEFT_CORRECTION");
    saveLatestReadings(pair, averageCm, diffCm, "LEFT");
    updateOled(pair, averageCm, diffCm, "LEFT");
    return;
  }

  // Left side is farther from the wall, so turn right to correct.
  if (diffCm < -ALIGN_TOLERANCE_CM) {
    rotateRight(TURN_PWM);
    Serial.println("Action: RIGHT_CORRECTION");
    saveLatestReadings(pair, averageCm, diffCm, "RIGHT");
    updateOled(pair, averageCm, diffCm, "RIGHT");
    return;
  }

  moveForward(drivePwm);
  Serial.println("Action: FORWARD");
  saveLatestReadings(pair, averageCm, diffCm, "FWD");
  updateOled(pair, averageCm, diffCm, "FWD");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setupMotorPins();
  setupUltrasonics();
  setupButton();
  setupOled();
  stopCar();
  printHelp();
}

void loop() {
  bool buttonPressed = isStartButtonPressed();
  if (buttonPressed && !lastButtonPressed && !autoParkingEnabled) {
    autoParkingEnabled = true;
    Serial.println("Auto parking enabled by button");
  }
  lastButtonPressed = buttonPressed;

  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  if (autoParkingEnabled) {
    runAutoParking();
  } else if (millis() - lastIdleSampleMs >= 120) {
    DistancePair pair = readDistances();
    float averageCm = -1.0f;
    float diffCm = 0.0f;
    if (pair.rightCm >= 0.0f && pair.leftCm >= 0.0f) {
      averageCm = (pair.rightCm + pair.leftCm) * 0.5f;
      diffCm = pair.rightCm - pair.leftCm;
    }
    saveLatestReadings(pair, averageCm, diffCm, "READY");
    updateOled(pair, averageCm, diffCm, "READY");
    lastIdleSampleMs = millis();
  }
}
