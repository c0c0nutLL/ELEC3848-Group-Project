#include <Arduino.h>

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

// Update these pins to match your actual HC-SR04 wiring.
const uint8_t ULTRASONIC_TRIG_PIN = 22;
const uint8_t ULTRASONIC_ECHO_PIN = 23;

const float TARGET_DISTANCE_CM = 8.0f;
const float SLOWDOWN_DISTANCE_CM = 20.0f;
const float DISTANCE_TOLERANCE_CM = 0.8f;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

int fastPwm = 170;
int slowPwm = 110;
bool autoParkingEnabled = false;
unsigned long lastTelemetryMs = 0;

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

void moveBackward(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
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

void setupUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f * 0.5f;
}

void printHelp() {
  Serial.println("Ultrasonic parking test ready.");
  Serial.println("Commands:");
  Serial.println("  G - start auto parking");
  Serial.println("  X - stop auto parking");
  Serial.println("  F - manual forward");
  Serial.println("  B - manual backward");
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
      moveForward(fastPwm);
      Serial.println("Manual forward");
      break;
    case 'B':
    case 'b':
      autoParkingEnabled = false;
      moveBackward(fastPwm);
      Serial.println("Manual backward");
      break;
    case 'S':
    case 's':
      autoParkingEnabled = false;
      stopCar();
      Serial.println("Stop");
      break;
    case 'D':
    case 'd':
      Serial.print("Distance(cm): ");
      Serial.println(readDistanceCm());
      break;
    default:
      break;
  }
}

void runAutoParking() {
  float distanceCm = readDistanceCm();
  if (distanceCm < 0.0f) {
    stopCar();
    Serial.println("Ultrasonic timeout");
    return;
  }

  if (millis() - lastTelemetryMs >= 200) {
    Serial.print("Distance(cm): ");
    Serial.println(distanceCm);
    lastTelemetryMs = millis();
  }

  if (distanceCm <= TARGET_DISTANCE_CM + DISTANCE_TOLERANCE_CM) {
    stopCar();
    autoParkingEnabled = false;
    Serial.println("Parking complete");
    return;
  }

  if (distanceCm <= SLOWDOWN_DISTANCE_CM) {
    moveForward(slowPwm);
  } else {
    moveForward(fastPwm);
  }
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupUltrasonic();
  stopCar();
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  if (autoParkingEnabled) {
    runAutoParking();
  }
}
