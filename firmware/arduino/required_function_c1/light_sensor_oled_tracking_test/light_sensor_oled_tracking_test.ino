#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28

#define SERVO_PIN 9
#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A3
#define TOL 50
#define K 5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo myservo;

int int_position = 90;

// Calibration variables copied from the original program.
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;
int int_left, int_right;

void updateOled(const char *line1, int leftValue, int rightValue, int servoSpeed) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.print("L:");
  display.print(leftValue);
  display.print(" R:");
  display.println(rightValue);
  display.print("Servo:");
  display.println(servoSpeed);
  display.display();
}

void showMessage(const char *line1, const char *line2, const char *line3 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.println(line2);
  display.println(line3);
  display.display();
}

int computeIntensity(int rawValue, int darkValue, int slope) {
  if (slope == 0) {
    return 0;
  }

  int intensity = (rawValue - darkValue) / slope;
  return constrain(intensity, 0, 100);
}

void tracker() {
  // If left sensor is brighter than right sensor, decrease servo angle and turn LEFT.
  if (int_left > (int_right + TOL)) {
    if (int_position > 0) {
      int_position = int_position - K;
      myservo.write(int_position);
    }
  }

  // If right sensor is brighter than left sensor, increase servo angle and turn RIGHT.
  if (int_left < (int_right - TOL)) {
    if (int_position < 180) {
      int_position = int_position + K;
      myservo.write(int_position);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Program started"));
  Wire.begin();
  Serial.println(F("Wire begin already"));

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  Serial.println(F("After If "));
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("AI Robot");
  display.display();

  myservo.attach(SERVO_PIN);
  myservo.write(int_position);
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  Serial.println();

  // Measure the sensors reading at ambient light intensity.
  Serial.println("Calibration in progress, put the sensors under the light (~ 5 sec) ......");
  Serial.println("***********************");
  showMessage("Calibration", "Light source", "Wait 5 sec");
  delay(5000);

  int_adc0 = analogRead(LEFT_SENSOR_PIN);
  int_adc1 = analogRead(RIGHT_SENSOR_PIN);
  Serial.print("Left : ");
  Serial.println(int_adc0);
  Serial.print("Right : ");
  Serial.println(int_adc1);
  showMessage("Ambient done", "Left/Right read", "Check Serial");
  delay(1000);

  Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
  Serial.println("************ Put Fingers *****************");
  showMessage("Cover sensors", "with fingers", "Wait 5 sec");
  delay(5000);
  Serial.println("********* START Calibration **************");

  // Measure the sensors reading at zero light intensity.
  int_adc0_c = analogRead(LEFT_SENSOR_PIN);
  int_adc1_c = analogRead(RIGHT_SENSOR_PIN);

  // Calculate the slope of light intensity to ADC reading equations.
  int_adc0_m = (int_adc0 - int_adc0_c) / 100;
  int_adc1_m = (int_adc1 - int_adc1_c) / 100;
  showMessage("Calibrating", "Do not move", "Wait 10 sec");
  delay(10000);

  Serial.println("\n******** Completed! Remove your hands ********");
  showMessage("Calibration", "Completed", "Remove hands");
  delay(4000);
}

void loop() {
  int_left = computeIntensity(analogRead(LEFT_SENSOR_PIN), int_adc0_c, int_adc0_m);
  int_right = computeIntensity(analogRead(RIGHT_SENSOR_PIN), int_adc1_c, int_adc1_m);

  Serial.print("Left sensor intensity = ");
  Serial.print(int_left);
  Serial.print(";  Right sensor intensity = ");
  Serial.print(int_right);
  Serial.print(";  Servo Speed (-90 to 90) = ");
  Serial.println(int_position - 90);

  updateOled("Light Sensors", int_left, int_right, int_position - 90);
  tracker();
  delay(100);
}
