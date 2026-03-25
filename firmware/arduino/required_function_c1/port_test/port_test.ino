#include <Arduino.h>

// 这个程序用于测试某个 Arduino 引脚是否能正常输出高电平。
// 把 TEST_PIN 改成你想测的引脚编号，烧录后用万用表测这个引脚对 GND 的电压。
// 如果引脚正常工作，通常会测到接近 5V 的电压。

const uint8_t TEST_PIN = 33;

void setup() {
  pinMode(TEST_PIN, OUTPUT);
  digitalWrite(TEST_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("Port test started");
  Serial.print("TEST_PIN = ");
  Serial.println(TEST_PIN);
  Serial.println("Output set to HIGH");
}

void loop() {
  // 持续保持高电平，避免被其他逻辑改写。
  digitalWrite(TEST_PIN, HIGH);
  delay(1000);
}
