

# include "MyServo.h"
# include "Kinematics.h"

MyServo servo0(1, 1);
MyServo servo1(1, 1);

Kinematics kinematics;

void setup() {
  // put your setup code here, to run once:

  servo0.attach(9);
  servo1.attach(10);

  Kinematics::arrWrap4 pose = kinematics.getPosition(10, 20, 20, 1, 1, 0);

  Serial.begin(9600);
  kinematics.printMatrix(Serial, "end effector pose: ", pose);

  Kinematics::arrWrap1 jointAngles = kinematics.getAngles(1.92, 0.41, 0.35, 1, 1, 0);
  kinematics.printMatrix(Serial, "joint angles: ", jointAngles);

  double x = 1001;
  Serial.println(x);
}

void loop() { ///////////////////////////////////////////////TODO fix camal case for parameter names
  // put your main code here, to run repeatedly:

  servo0.move(0);
  servo1.move(0);
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
