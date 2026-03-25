#include <Arduino.h>
#include <Wire.h>

// 这个程序用于扫描 I2C 总线上的设备地址。
// 如果 OLED 正常接在 I2C 上，通常会扫到 0x3C 或 0x3D。

unsigned long lastScanMs = 0;

void scanI2cBus() {
  Serial.println("I2C scan start");

  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("No I2C device found");
  }

  Serial.println("I2C scan end");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(500);
  scanI2cBus();
}

void loop() {
  if (millis() - lastScanMs >= 3000) {
    scanI2cBus();
    lastScanMs = millis();
  }
}
