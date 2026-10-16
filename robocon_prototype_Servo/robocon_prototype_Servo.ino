
#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 18;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 360;

int currentAngle = 0;

void moveServo(int angle) {
  currentAngle = constrain(angle, MIN_ANGLE, MAX_ANGLE);

  // Studica 75002 nominal 300-degree PWM range
  int pulse = map(currentAngle, 0, 300, 500, 2500);

  myServo.writeMicroseconds(pulse);

  Serial.print("ANGLE:");
  Serial.println(currentAngle);
}

void setup() {
  Serial.begin(115200);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);

  moveServo(0);
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("A:")) {
      int angle = input.substring(2).toInt();
      moveServo(angle);
    }
  }
}
