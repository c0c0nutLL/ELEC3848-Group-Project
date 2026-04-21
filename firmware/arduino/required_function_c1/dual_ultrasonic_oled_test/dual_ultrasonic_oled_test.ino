#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
  OBSOLETE / HISTORICAL REFERENCE

  This sketch was written for the removed side-ultrasonic experiment
  (left side + right side wall-distance observation).
  Current hardware no longer installs those two side sensors.
*/

// 电机引脚沿用 reference.ino
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

// 右侧超声波 ultrasonic1
const uint8_t ULTRASONIC_RIGHT_TRIG_PIN = 30;
const uint8_t ULTRASONIC_RIGHT_ECHO_PIN = 32;

// 左侧超声波 ultrasonic2
const uint8_t ULTRASONIC_LEFT_TRIG_PIN = 28;
const uint8_t ULTRASONIC_LEFT_ECHO_PIN = 25;

// 按钮与颜色传感器引脚沿用之前的文件
const uint8_t START_BUTTON_PIN = 33;
const uint8_t COLOR_OUT_PIN = 46;
const uint8_t COLOR_S0_PIN = 44;
const uint8_t COLOR_S1_PIN = 45;
const uint8_t COLOR_S2_PIN = 13;
const uint8_t COLOR_S3_PIN = 11;
const uint8_t COLOR_LED_PIN = 47;

const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;
const unsigned long SAMPLE_INTERVAL_MS = 120UL;
const unsigned long COLOR_SEARCH_TIMEOUT_MS = 4000UL;
const unsigned long COLOR_SAMPLE_INTERVAL_MS = 70UL;
const float STRAFE_DEADBAND_CM = 2.0f;

const int STRAFE_PWM = 32;
const uint16_t RED_GREEN_GAP_FOR_RED = 12;
const uint16_t RED_GREEN_GAP_FOR_GREEN = 3;
const uint8_t REQUIRED_COLOR_STREAK = 3;

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const int8_t OLED_RESET = 28;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum DetectedColor {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_NO_SIGNAL
};

// 放在所有函数前面，避免 Arduino 自动生成原型时找不到类型
struct ColorReading {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
};

unsigned long lastSampleMs = 0;
unsigned long lastColorSampleMs = 0;
unsigned long searchStartMs = 0;

bool lastButtonPressed = false;
bool searchActive = false;

float rightCm = -1.0f;
float leftCm = -1.0f;
float diffCm = 0.0f;

DetectedColor rawColor = COLOR_UNKNOWN;
DetectedColor stableColor = COLOR_UNKNOWN;
DetectedColor lastRawColor = COLOR_UNKNOWN;
uint8_t colorStreak = 0;

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

void updateStableColor(DetectedColor newRawColor) {
  rawColor = newRawColor;

  if (rawColor == lastRawColor) {
    if (colorStreak < 255) {
      colorStreak++;
    }
  } else {
    lastRawColor = rawColor;
    colorStreak = 1;
  }

  if (colorStreak >= REQUIRED_COLOR_STREAK) {
    stableColor = rawColor;
  }
}

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
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

bool isButtonPressed() {
  return digitalRead(START_BUTTON_PIN) == LOW;
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

const char *actionName() {
  if (!searchActive) {
    return "READY";
  }
  if (stableColor == COLOR_RED || stableColor == COLOR_GREEN) {
    return "FOUND";
  }
  if (diffCm > STRAFE_DEADBAND_CM) {
    return "RIGHT";
  }
  if (diffCm < -STRAFE_DEADBAND_CM) {
    return "LEFT";
  }
  return "HOLD";
}

void printFloatOrDash(float value) {
  if (value < 0.0f) {
    display.print("--");
  } else {
    display.print(value, 1);
  }
}

void updateOled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("R:");
  printFloatOrDash(rightCm);
  display.print(" L:");
  printFloatOrDash(leftCm);
  display.println();

  display.print("D:");
  if (rightCm < 0.0f || leftCm < 0.0f) {
    display.print("--");
  } else {
    display.print(diffCm, 1);
  }
  display.print(" ");
  display.println(actionName());

  display.print("Color:");
  display.println(colorName(stableColor));

  if (searchActive) {
    display.print("T:");
    display.print((millis() - searchStartMs) / 1000.0f, 1);
    display.println("s");
  } else {
    display.println("T:idle");
  }

  display.display();
}

void setup() {
  Serial.begin(115200);

  Serial.println("OBSOLETE: side ultrasonic hardware was removed on 2026-04-21.");
  Serial.println("Keep this sketch only as historical reference.");

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
  stopCar();

  pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_RIGHT_TRIG_PIN, LOW);
  digitalWrite(ULTRASONIC_LEFT_TRIG_PIN, LOW);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  pinMode(COLOR_S0_PIN, OUTPUT);
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  pinMode(COLOR_LED_PIN, OUTPUT);
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);
  digitalWrite(COLOR_LED_PIN, HIGH);

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLD");
  display.display();
  delay(500);
}

void loop() {
  bool buttonPressed = isButtonPressed();
  if (buttonPressed && !lastButtonPressed && !searchActive) {
    searchActive = true;
    searchStartMs = millis();
    stableColor = COLOR_UNKNOWN;
    rawColor = COLOR_UNKNOWN;
    lastRawColor = COLOR_UNKNOWN;
    colorStreak = 0;
    Serial.println("Search start");
  }
  lastButtonPressed = buttonPressed;

  if (millis() - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = millis();
    rightCm = readDistanceCm(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
    delay(4);
    leftCm = readDistanceCm(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);

    if (rightCm >= 0.0f && leftCm >= 0.0f) {
      diffCm = rightCm - leftCm;
    } else {
      diffCm = 0.0f;
    }
  }

  if (searchActive && millis() - lastColorSampleMs >= COLOR_SAMPLE_INTERVAL_MS) {
    lastColorSampleMs = millis();
    updateStableColor(classifyColor(readColor()));
  }

  if (searchActive) {
    if (stableColor == COLOR_RED || stableColor == COLOR_GREEN) {
      stopCar();
      searchActive = false;
      Serial.print("Color found: ");
      Serial.println(colorName(stableColor));
    } else if (millis() - searchStartMs >= COLOR_SEARCH_TIMEOUT_MS) {
      stopCar();
      searchActive = false;
      Serial.println("Search timeout");
    } else if (diffCm > STRAFE_DEADBAND_CM) {
      strafeRight(STRAFE_PWM);
    } else if (diffCm < -STRAFE_DEADBAND_CM) {
      strafeLeft(STRAFE_PWM);
    } else {
      stopCar();
    }
  } else {
    stopCar();
  }

  updateOled();

  Serial.print("Right=");
  Serial.print(rightCm, 1);
  Serial.print(" Left=");
  Serial.print(leftCm, 1);
  Serial.print(" Diff=");
  Serial.print(diffCm, 1);
  Serial.print(" Raw=");
  Serial.print(colorName(rawColor));
  Serial.print(" Stable=");
  Serial.print(colorName(stableColor));
  Serial.print(" Action=");
  Serial.println(actionName());
}
