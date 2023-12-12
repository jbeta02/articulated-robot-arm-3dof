

# include "MyServo.h"
# include "Kinematics.h"

MyServo servo1;
MyServo servo2;
MyServo servo3;

Kinematics kinematics;
Kinematics::arrWrap4 pose;
Kinematics::arrWrap1 jointAngles;

void setup() {
  // put your setup code here, to run once:

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

  servo3.shiftOrigin(180);

  pose = kinematics.getPosition(0, 90, -90, 1, 1, 0);

  Serial.begin(9600);
  kinematics.printMatrix(Serial, "end effector pose: ", pose);

  jointAngles = kinematics.getAngles(0, 0, 2, 1, 1, 0);
  kinematics.printMatrix(Serial, "joint angles (degrees): ", jointAngles);
}

void loop() { ///////////////////////////////////////////////TODO fix camal case for parameter names and make code follow c++ coding conventions 
  // put your main code here, to run repeatedly:

  servo1.move(jointAngles.arr[0][0]);
  servo2.move(jointAngles.arr[1][0]);
  servo3.move(jointAngles.arr[2][0]);
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
