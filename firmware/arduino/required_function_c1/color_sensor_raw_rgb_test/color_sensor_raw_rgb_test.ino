#include <Arduino.h>

// 这个程序用于读取颜色传感器的原始 RGB 数据。
// 这里默认 GY-31 使用常见的 TCS3200/TCS230 接口。
// 请根据实际接线修改下面的引脚定义。

const uint8_t COLOR_S0_PIN = 44;
const uint8_t COLOR_S1_PIN = 45;
const uint8_t COLOR_S2_PIN = 13;
const uint8_t COLOR_S3_PIN = 11;
const uint8_t COLOR_OUT_PIN = 46;
const uint8_t COLOR_LED_PIN = 47;

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

  // 输出频率缩放到 20%，便于读取。
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);

  // 默认打开补光灯。
  digitalWrite(COLOR_LED_PIN, HIGH);
}

// 选择一个颜色通道并读取对应脉冲宽度。
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

// 依次读取红、蓝、绿三个通道的原始值。
ColorReading readColor() {
  ColorReading reading;

  reading.red = readColorChannel(false, false);
  reading.blue = readColorChannel(false, true);
  reading.green = readColorChannel(true, true);

  return reading;
}

void printHelp() {
  Serial.println("Color sensor raw RGB test ready.");
  Serial.println("Commands:");
  Serial.println("  R - 读取一次 RGB");
  Serial.println("  C - 持续输出 RGB");
  Serial.println("  S - 停止持续输出");
}

bool continuousMode = true;
unsigned long lastPrintMs = 0;

void printSample() {
  ColorReading reading = readColor();

  Serial.print("R=");
  Serial.print(reading.red);
  Serial.print(" G=");
  Serial.print(reading.green);
  Serial.print(" B=");
  Serial.println(reading.blue);
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
