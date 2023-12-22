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
  if (angle < minAngle) {
    angle = minAngle;
    int targetAngle = int(round((angle + originShift) * position_to_angle));
    servo.write(targetAngle);
  }
  if (angle > maxAngle) {
    angle = maxAngle;
    int targetAngle = int(round((angle + originShift) * position_to_angle));
    servo.write(targetAngle);
  }
  else {
    int targetAngle = int(round((angle + originShift) * position_to_angle));
    servo.write(targetAngle);
  }
}

// set servo input bounds (not needed since servo will aleady get as close to target as it can even if target oob)
void MyServo::setBounds(float minAngle, float maxAngle) {
  this->minAngle = minAngle;
  this->maxAngle = maxAngle;
}

// shift origin to allow for negative values that will fit within servo coordinate system and bounds
void MyServo::shiftOrigin(float originShift) {
  this->originShift = originShift;
}