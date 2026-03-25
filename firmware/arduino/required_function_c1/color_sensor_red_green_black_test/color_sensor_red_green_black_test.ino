#include <Arduino.h>

// 这个测试程序用于读取 GY-31 颜色传感器，并尝试区分红色、绿色和黑色。
// 这里默认 GY-31 使用的是常见的 TCS3200/TCS230 接口。

// 电机引脚定义沿用 reference.ino，方便直接在当前底盘上测试颜色触发动作。
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

// 这些引脚定义需要按照你们实际接线修改。
const uint8_t COLOR_S0_PIN = 44;
const uint8_t COLOR_S1_PIN = 45;
const uint8_t COLOR_S2_PIN = 13;
const uint8_t COLOR_S3_PIN = 11;
const uint8_t COLOR_OUT_PIN = 46;
const uint8_t COLOR_LED_PIN = 47;  // 如果模块带补光 LED 控制脚，可以在这里控制开关。

// 这些阈值根据你提供的红纸和绿纸样本数据设置。
// 140 对当前底盘可能不足以起步，这里提高到和基础电机测试更接近的值。
const int MOTOR_PWM = 180;
const uint16_t RED_GREEN_GAP_FOR_RED = 12;
const uint16_t RED_GREEN_GAP_FOR_GREEN = 3;
const uint8_t REQUIRED_STREAK = 3;

struct ColorReading {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
};

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

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

// 按之前测试文件的定义，全部 FORWARD 作为右旋。
void rotateRight() {
  MOTORA_FORWARD(MOTOR_PWM);
  MOTORB_FORWARD(MOTOR_PWM);
  MOTORC_FORWARD(MOTOR_PWM);
  MOTORD_FORWARD(MOTOR_PWM);
}

// 按之前测试文件的定义，全部 BACKOFF 作为左旋。
void rotateLeft() {
  MOTORA_BACKOFF(MOTOR_PWM);
  MOTORB_BACKOFF(MOTOR_PWM);
  MOTORC_BACKOFF(MOTOR_PWM);
  MOTORD_BACKOFF(MOTOR_PWM);
}

// UNKNOWN 时默认保持直行。
void moveForward() {
  MOTORA_FORWARD(MOTOR_PWM);
  MOTORB_BACKOFF(MOTOR_PWM);
  MOTORC_FORWARD(MOTOR_PWM);
  MOTORD_BACKOFF(MOTOR_PWM);
}

// 初始化颜色传感器，并设置输出频率缩放。
void setupColorSensor() {
  pinMode(COLOR_S0_PIN, OUTPUT);
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  pinMode(COLOR_LED_PIN, OUTPUT);

  // 将 TCS3200 的输出频率缩放到 20%，便于 Arduino 读取。
  digitalWrite(COLOR_S0_PIN, HIGH);
  digitalWrite(COLOR_S1_PIN, LOW);

  // 如果模块支持补光 LED 控制，这里默认打开。
  digitalWrite(COLOR_LED_PIN, HIGH);
}

// 选择一个颜色通道并读取其脉冲宽度。
// 对 TCS3200 来说，脉冲越短，通常表示该颜色反射越强。
uint16_t readColorChannel(bool s2, bool s3) {
  digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
  delay(3);

  unsigned long pulse = pulseIn(COLOR_OUT_PIN, LOW, 50000UL);
  if (pulse == 0) {
    return 0;
  }

  // TCS3200 输出的是频率信号，这里用脉冲宽度近似表示颜色强度。
  return static_cast<uint16_t>(pulse);
}

// 依次读取红、蓝、绿三个通道。
ColorReading readColor() {
  ColorReading reading;

  // 常见 TCS3200 颜色滤波选择：
  // red:   S2 LOW,  S3 LOW
  // blue:  S2 LOW,  S3 HIGH
  // clear: S2 HIGH, S3 LOW
  // green: S2 HIGH, S3 HIGH
  reading.red = readColorChannel(false, false);
  reading.blue = readColorChannel(false, true);
  reading.green = readColorChannel(true, true);
  return reading;
}

// 根据三个通道的读数做一个简单分类。
// 当前版本只分 RED、GREEN 和 UNKNOWN，不再分 BLACK。
const char *classifyColor(const ColorReading &reading) {
  if (reading.red == 0 || reading.green == 0 || reading.blue == 0) {
    return "NO_SIGNAL";
  }

  if (reading.red + RED_GREEN_GAP_FOR_RED < reading.green) {
    return "RED";
  }

  if (reading.green + RED_GREEN_GAP_FOR_GREEN < reading.red) {
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
  Serial.println("  F - manual forward");
  Serial.println("  L - manual rotate left");
  Serial.println("  E - manual rotate right");
  Serial.println("  X - manual stop");
}

bool continuousMode = true;
unsigned long lastPrintMs = 0;
const char *stableLabel = "UNKNOWN";
const char *lastRawLabel = "UNKNOWN";
uint8_t labelStreak = 0;

// 打印一次原始读数和分类结果，便于后续做阈值校准。
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

// 连续多次得到相同分类后，才把它作为稳定结果输出，减少抖动。
const char *updateStableLabel(const char *rawLabel) {
  if (strcmp(rawLabel, lastRawLabel) == 0) {
    if (labelStreak < 255) {
      labelStreak++;
    }
  } else {
    lastRawLabel = rawLabel;
    labelStreak = 1;
  }

  if (labelStreak >= REQUIRED_STREAK) {
    if (strcmp(rawLabel, "RED") == 0 || strcmp(rawLabel, "GREEN") == 0) {
      stableLabel = rawLabel;
    } else if (strcmp(rawLabel, "UNKNOWN") == 0 || strcmp(rawLabel, "NO_SIGNAL") == 0) {
      // UNKNOWN 时不立刻切回直行，保持上一个稳定动作。
      // 只有系统刚启动、还没有稳定识别过颜色时，才维持 UNKNOWN。
      if (strcmp(stableLabel, "UNKNOWN") == 0) {
        stableLabel = "UNKNOWN";
      }
    }
  }

  return stableLabel;
}

// 根据稳定颜色分类直接驱动底盘：
// RED -> 右旋
// GREEN -> 左旋
// UNKNOWN -> 直行
void driveByColor(const char *label) {
  if (strcmp(label, "RED") == 0) {
    rotateRight();
  } else if (strcmp(label, "GREEN") == 0) {
    rotateLeft();
  } else {
    moveForward();
  }
}

// 串口命令用于切换单次读取和持续输出模式。
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
    case 'F':
    case 'f':
      continuousMode = false;
      moveForward();
      Serial.println("Manual forward");
      break;
    case 'L':
    case 'l':
      continuousMode = false;
      rotateLeft();
      Serial.println("Manual rotate left");
      break;
    case 'E':
    case 'e':
      continuousMode = false;
      rotateRight();
      Serial.println("Manual rotate right");
      break;
    case 'X':
    case 'x':
      continuousMode = false;
      stopCar();
      Serial.println("Manual stop");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupColorSensor();
  stopCar();
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  // 持续模式下定时输出，方便直接观察不同颜色的原始值变化。
  if (continuousMode && millis() - lastPrintMs >= 250) {
    ColorReading reading = readColor();
    const char *rawLabel = classifyColor(reading);
    const char *label = updateStableLabel(rawLabel);

    Serial.print("R=");
    Serial.print(reading.red);
    Serial.print(" G=");
    Serial.print(reading.green);
    Serial.print(" B=");
    Serial.print(reading.blue);
    Serial.print(" -> raw=");
    Serial.print(rawLabel);
    Serial.print(" stable=");
    Serial.println(label);

    // 串口里同步打印当前动作，便于确认程序是否真的进入了对应分支。
    driveByColor(label);
    Serial.print("Action: ");
    if (strcmp(label, "RED") == 0) {
      Serial.println("ROTATE_RIGHT");
    } else if (strcmp(label, "GREEN") == 0) {
      Serial.println("ROTATE_LEFT");
    } else {
      Serial.println("FORWARD");
    }
    lastPrintMs = millis();
  }
}
