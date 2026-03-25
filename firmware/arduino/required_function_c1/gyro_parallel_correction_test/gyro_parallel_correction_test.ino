#include <Arduino.h>
#include <Wire.h>

// 这个测试程序用于验证“陀螺仪平行矫正”逻辑。
// 程序会从 MPU6050 读取角速度，积分得到当前航向角，
// 然后控制底盘旋转回目标角度，用于测试停车后的平行修正。

// Motor pin mapping copied from Car_Volt_Feedback_24A.ino
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

const uint8_t MPU6050_ADDR = 0x68;
const float GYRO_SCALE_DPS_PER_LSB = 131.0f;  // FS_SEL = 0, +-250 deg/s
const float HEADING_TOLERANCE_DEG = 1.5f;
const int ROTATE_PWM = 120;

// headingDeg 表示当前相对零点的航向角。
// targetHeadingDeg 表示希望矫正回去的目标角度。
float gyroZBias = 0.0f;
float headingDeg = 0.0f;
float targetHeadingDeg = 0.0f;
unsigned long lastUpdateMicros = 0;
unsigned long lastTelemetryMs = 0;
bool correctionEnabled = false;

void stopCar() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

void rotateLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
}

void rotateRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
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

// 向 MPU6050 写寄存器的简单封装。
void writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// 通过 I2C 从 MPU6050 连续读取若干字节。
bool readMpuBytes(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(MPU6050_ADDR, count);
  if (received != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

// 唤醒 MPU6050，并把陀螺仪量程设置为 +-250 deg/s，
// 这样做小角度修正时分辨率更高。
bool initMpu6050() {
  Wire.begin();
  writeMpuRegister(0x6B, 0x00);  // wake up
  writeMpuRegister(0x1B, 0x00);  // gyro +-250 deg/s
  delay(100);

  uint8_t whoAmI = 0;
  if (!readMpuBytes(0x75, 1, &whoAmI)) {
    return false;
  }
  return whoAmI == 0x68;
}

// 读取 Z 轴角速度，单位是度每秒。
float readGyroZDps() {
  uint8_t buffer[2];
  if (!readMpuBytes(0x47, 2, buffer)) {
    return 0.0f;
  }

  int16_t rawGyroZ = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  return static_cast<float>(rawGyroZ) / GYRO_SCALE_DPS_PER_LSB;
}

// 在小车静止时估计陀螺仪零偏，减少后续积分漂移。
void calibrateGyroBias() {
  const int sampleCount = 500;
  float sum = 0.0f;

  stopCar();
  Serial.println("Calibrating gyro. Keep the robot still.");
  delay(500);

  for (int i = 0; i < sampleCount; ++i) {
    sum += readGyroZDps();
    delay(4);
  }

  gyroZBias = sum / sampleCount;
  headingDeg = 0.0f;
  targetHeadingDeg = 0.0f;
  Serial.print("Gyro bias(dps): ");
  Serial.println(gyroZBias, 4);
}

// 对 Z 轴角速度做时间积分，得到当前航向角估计值。
void updateHeading() {
  unsigned long now = micros();
  if (lastUpdateMicros == 0) {
    lastUpdateMicros = now;
    return;
  }

  float dt = (now - lastUpdateMicros) / 1000000.0f;
  lastUpdateMicros = now;

  float gyroZ = readGyroZDps() - gyroZBias;
  headingDeg += gyroZ * dt;
}

// 根据当前误差控制底盘左转或右转，直到误差进入容差范围。
void runCorrection() {
  float errorDeg = targetHeadingDeg - headingDeg;

  if (errorDeg > HEADING_TOLERANCE_DEG) {
    rotateRight(ROTATE_PWM);
  } else if (errorDeg < -HEADING_TOLERANCE_DEG) {
    rotateLeft(ROTATE_PWM);
  } else {
    stopCar();
    correctionEnabled = false;
    Serial.println("Parallel correction complete");
  }
}

void printHelp() {
  Serial.println("Gyro parallel correction test ready.");
  Serial.println("Commands:");
  Serial.println("  C - calibrate gyro bias");
  Serial.println("  Z - zero current heading");
  Serial.println("  T - set current heading as target");
  Serial.println("  P - start correction to target heading");
  Serial.println("  X - stop correction");
  Serial.println("  H - print heading now");
}

// 串口命令用于测试时手动触发校准、清零、记录目标角和启动修正。
void handleCommand(char cmd) {
  switch (cmd) {
    case 'C':
    case 'c':
      calibrateGyroBias();
      break;
    case 'Z':
    case 'z':
      headingDeg = 0.0f;
      targetHeadingDeg = 0.0f;
      Serial.println("Heading zeroed");
      break;
    case 'T':
    case 't':
      targetHeadingDeg = headingDeg;
      Serial.print("Target heading set to ");
      Serial.println(targetHeadingDeg, 2);
      break;
    case 'P':
    case 'p':
      correctionEnabled = true;
      Serial.println("Parallel correction enabled");
      break;
    case 'X':
    case 'x':
      correctionEnabled = false;
      stopCar();
      Serial.println("Parallel correction disabled");
      break;
    case 'H':
    case 'h':
      Serial.print("Heading(deg): ");
      Serial.println(headingDeg, 2);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  stopCar();

  if (!initMpu6050()) {
    Serial.println("MPU6050 init failed");
    while (true) {
      delay(1000);
    }
  }

  calibrateGyroBias();
  printHelp();
}

void loop() {
  // 即使当前没有启动修正，也持续更新航向角估计。
  updateHeading();

  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  if (correctionEnabled) {
    runCorrection();
  }

  if (millis() - lastTelemetryMs >= 200) {
    Serial.print("Heading(deg): ");
    Serial.println(headingDeg, 2);
    lastTelemetryMs = millis();
  }
}
