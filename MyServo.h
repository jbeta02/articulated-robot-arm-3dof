// header file for MyServo

#ifndef MyServo_h
#define MyServo_h
#include <Arduino.h>
#include <Servo.h>
#include <math.h>


class MyServo {
  
  public:

    MyServo(int maxPosition, int maxAngle);

    void attach(byte pin);

    void move(float angle);


  private: 
    Servo servo;

    int _maxPosition;
    int _maxAngle;
    // obtained from testing and looking at servo specs
    // servo can turn from 0 to 270 but corresponds to 0 to 180 servo write behavior
    float position_to_angle = 2.0/3.0;
    
};

#endif