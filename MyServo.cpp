// source file for MyServo

#include <Arduino.h>
#include <Servo.h>
#include "MyServo.h"
#include <math.h>



MyServo::MyServo() {
  
}

void MyServo::attach(byte pin) {
  servo.attach(pin);
}

void MyServo::move(float angle) {
  int targetAngle = int(round(angle * position_to_angle));
  servo.write(targetAngle);
}

// will use this and move to make it possible to move angle2 in negative direction
void MyServo::setBounds(float minAngle, float maxAngle) {
  this->minAngle = minAngle;
  this->maxAngle = maxAngle;
}