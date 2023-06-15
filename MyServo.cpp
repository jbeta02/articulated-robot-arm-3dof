// source file for MyServo

#include <Arduino.h>
#include <Servo.h>
#include "MyServo.h"
#include <math.h>



MyServo::MyServo(int maxPosition, int maxAngle) {
  _maxPosition = maxPosition;
  _maxAngle = maxAngle;
}

void MyServo::attach(byte pin) {
  servo.attach(pin);
}

void MyServo::move(float angle) {
  int targetAngle = int(round(angle * position_to_angle));
  servo.write(targetAngle);
}