

# include "MyServo.h"
# include "Kinematics.h"

MyServo handServo;
MyServo servo1;
MyServo servo2;
MyServo servo3;

Kinematics kinematics;
Kinematics::arrWrap4 pose;
Kinematics::arrWrap1 jointAngles;

void setup() {
  // put your setup code here, to run once:

  handServo.attach(8); // max position is 160 (translates to about 180, servo not very accurate)
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

  handServo.setBounds(0, 75);
  servo1.setBounds(0, 180);
  servo2.setBounds(-60, 0);
  servo3.setBounds(0, 90);

  servo3.shiftOrigin(180); // shift origin to allow negitive angle inputs

  pose = kinematics.getPosition(0, 90, -90, 1, 1, 0);

  Serial.begin(9600);
  kinematics.printMatrix(Serial, "end effector pose: ", pose);

  jointAngles = kinematics.getAngles(0, 0, 2, 1, 1, 0);
  kinematics.printMatrix(Serial, "joint angles (degrees): ", jointAngles);
}

void loop() { ///////////////////////////////////////////////TODO fix camal case for parameter names and make code follow c++ coding conventions 
  // put your main code here, to run repeatedly:

  // servo1.move(jointAngles.arr[0][0]);
  // servo2.move(jointAngles.arr[1][0]);
  // servo3.move(jointAngles.arr[2][0]);
  
  handServo.move(0);
  // servo1.move(0);
  // servo2.move(0);
  // servo3.move(0);
  delay(2000);

  // servo0.move(30);
  // servo1.move(30);
  // delay(2000);

  // servo0.move(90);
  // servo1.move(90);
  // delay(2000);

  // servo0.move(90);
  // servo1.move(90);
  // delay(2000);

}
