#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 这个程序用于单独测试 OLED 是否能正常初始化和持续刷新。
// 配置参考 reference.ino：
// - SSD1306
// - I2C 地址 0x3C
// - OLED_RESET = 28

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned long lastUpdateMs = 0;
int counter = 0;

void showBootScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED test");
  display.println("Init OK");
  display.display();
}

void showLoopScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED running");
  display.print("Count: ");
  display.println(counter);
  display.print("Time: ");
  display.println(millis() / 1000);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("SSD1306 init OK");
  showBootScreen();
}

void loop() {
  if (millis() - lastUpdateMs >= 500) {
    counter++;
    showLoopScreen();

    Serial.print("OLED update ");
    Serial.println(counter);

    lastUpdateMs = millis();
  }
}
