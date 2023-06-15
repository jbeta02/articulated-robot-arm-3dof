

# include "MyServo.h"
# include "Kinematics.h"

MyServo servo0(1, 1);
MyServo servo1(1, 1);

Kinematics kinematics;

void setup() {
  // put your setup code here, to run once:

  servo0.attach(9);
  servo1.attach(10);

  Kinematics::arr4 pose = kinematics.getPosition(0, 0, 0, 1, 1, 0);

  Serial.begin(9600);
  Serial.println("finished");
  //kinematics.printMatrix(Serial, "4x4 matix", pose);
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