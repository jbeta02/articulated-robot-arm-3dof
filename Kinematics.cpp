// source file for Kinematics

#include "Kinematics.h"
#include <Arduino.h>
#include <math.h>


Kinematics::Kinematics() {

}


Kinematics::arrWrap4 Kinematics::getPosition(float angle1, float angle2, float angle3, float link_len1, float link_len2, float base_height) {
  // convert angles from dgrees to radians
  angle1 = toRadians(angle1);
  angle2 = toRadians(angle2);
  angle3 = toRadians(angle3);


  // projection of coordiante from 1 on coordinate frame 0
  const struct arrWrap3 projection1_on_0 = {{
      {1, 0, 0}, 
      {0, 0, -1}, 
      {0, 1, 0}
  }};


  // rotation matrices
  struct arrWrap3 R_01 = multiply(projection1_on_0, yRotation(angle1));
  struct arrWrap3 R_12 = multiply(identityMatrix, zRotation(angle2));
  struct arrWrap3 R_23 = multiply(identityMatrix, zRotation(angle3));

  // displacement vectors
  struct arrWrap1 d_01 = {{{0}, {0}, {base_height}}};
  struct arrWrap1 d_12 = {{{link_len1 * cos(angle2)}, {link_len2 * sin(angle2)}, {0}}};
  struct arrWrap1 d_23 = {{{link_len2 * cos(angle3)}, {link_len2 * sin(angle3)}, {0}}};

  // homogenous transformation matrices
  struct arrWrap4 htm_01 = getHTM(R_01, d_01);
  struct arrWrap4 htm_12 = getHTM(R_12, d_12);
  struct arrWrap4 htm_23 = getHTM(R_23, d_23);

  //Serial.begin(9600);
  // printMatrix(Serial, "R_01", R_01);
  // printMatrix(Serial, "R_12", R_12);
  // printMatrix(Serial, "R_23", R_23);
  // delay(1000);
  // printMatrix(Serial, "htm_01", htm_01);
  // printMatrix(Serial, "htm_12", htm_12);
  // printMatrix(Serial, "htm_23", htm_23);


  struct arrWrap4 htm_03  = multiply(multiply(htm_01, htm_12), htm_23);

  // struct arrWrap4 matrix1 = {
  //         {1, 0, 0, 0}, 
  //         {0, 1, 0, 0}, 
  //         {0, 0, 1, 0},
  //         {0, 0, 0, 1}
  // };

  // struct arrWrap4 matrix2 = {
  //         {12, 22, 12, 12}, 
  //         {42, 52, 62, 12}, 
  //         {22, 32, 52, 12},
  //         {22, 32, 52, 12}
  // };

  // struct arrWrap4 matrix4 = {
  //         {2, 2, 2, 2}, 
  //         {4, 5, 6, 1}, 
  //         {2, 3, 5, 1},
  //         {2, 3, 5, 1}
  // };

  // struct arrWrap4 matrix3 = multiply(matrix1, matrix2);

  //Serial.begin(2000000);
  // printMatrix(Serial, "matrix3", matrix3);

  // struct arrWrap4 matrix8 = multiply(matrix1, matrix2);

  //printMatrix(Serial, "matrix8", matrix8);

  //struct arrWrap4 matrix5 = multiply(matrix1, matrix4);


  return htm_03;
}

// implement inverse kinematics using analytical approach by graphical method and trig
Kinematics::arrWrap1 Kinematics::getAngles(float link_len1, float link_len2, float base_height, float end_x, float end_y, float end_z) {
  // using kinematic diagram then splitting diagram into 2 2d diagrams (top view and side view)
  // then use trig to find angles based on given end effector position

  // declare angles
  float angle1;
  float angle2;
  float angle3;

  // declare trig vars
  float bigTriBase;
  float bigTriHeight;
  float hypot;
  float phi1;
  float phi2;
  float phi3;


  // get angle1

  // deal with x position at angle1 at 90 degrees
  if (end_x == 0) {
    angle1 = toRadians(90);
  }

  // deal with x position being negative
  else if (end_x != abs(end_x) && end_x != 0 && end_y != 0) {
    angle1 = toRadians(180) + atan(end_y / end_x);
  }

  else {
    angle1 = atan(end_y / end_x);
  }

  // get angle2

  bigTriBase = sqrt(pow(end_x, 2) + pow(end_y, 2));
  bigTriHeight = end_z - base_height;
  phi2 = atan(bigTriHeight / bigTriBase);

  hypot = sqrt(pow(bigTriBase, 2) + pow(bigTriHeight, 2));
  phi1 = acos((pow(link_len2, 2) - pow(link_len1, 2) - pow(hypot, 2)) / -2 * link_len1 * hypot);

  angle2 = phi2 + phi1;

  // get angle3

  phi3 = acos((pow(hypot, 2) - pow(link_len1, 2) - pow(link_len2, 2)) / (-2 * link_len1 * link_len2));
  angle3 = -(toRadians(180) - phi3); //neg for elbow up (remove neg for elbow down)


  // convert angles to degrees
  angle1 = toDegrees(angle1);
  angle2 = toDegrees(angle2);
  angle3 = toDegrees(angle3);

  struct arrWrap1 angles = {{{angle1}, {angle2}, {angle3}}};

  return angles;
}

// homogenous transformation matrix
Kinematics::arrWrap4 Kinematics::getHTM(struct arrWrap3 rotationMatrix, struct arrWrap1 displacementVector) {
  struct arrWrap4 htm = {{
    {rotationMatrix.arr[0][0], rotationMatrix.arr[0][1], rotationMatrix.arr[0][2], displacementVector.arr[0][0]}, 
    {rotationMatrix.arr[1][0], rotationMatrix.arr[1][1], rotationMatrix.arr[1][2], displacementVector.arr[0][1]}, 
    {rotationMatrix.arr[2][0], rotationMatrix.arr[2][1], rotationMatrix.arr[2][2], displacementVector.arr[0][2]},
    {0, 0, 0, 1}
  }};

  return htm;
}


// rotation be about given axis
Kinematics::arrWrap3 Kinematics::xRotation(float angle) {
  struct arrWrap3 xRotationMatrix = {{
    {1, 0, 0}, 
    {0, cos(angle), -sin(angle)}, 
    {0, sin(angle), cos(angle)}
  }};

  return xRotationMatrix;
}

Kinematics::arrWrap3 Kinematics::yRotation(float angle) {  
  struct arrWrap3 yRotationMatrix = {{
    {cos(angle), 0, sin(angle)}, 
    {0, 1, 0}, 
    {-sin(angle), 0, cos(angle)}
  }};

  return yRotationMatrix;
}

Kinematics::arrWrap3 Kinematics::zRotation(float angle) {
  struct arrWrap3 zRotationMatrix = {{
    {cos(angle), -sin(angle), 0}, 
    {sin(angle), cos(angle), 0}, 
    {0, 0, 1}
  }};

  return zRotationMatrix;
}


// convert from degrees to radians
float Kinematics::toRadians(float degrees) {
  return (degrees * (M_PI / 180));
}

// convert from radians to degrees
float Kinematics::toDegrees(float radians) {
  return (radians * (180 / M_PI));
}

// multiply two 3x3 matrices
Kinematics::arrWrap3 Kinematics::multiply(struct arrWrap3 matrixA, struct arrWrap3 matrixB) {
  // declare and initialize final matrix
  struct arrWrap3 matrixAB = {{
      {0, 0, 0}, 
      {0, 0, 0}, 
      {0, 0, 0}
  }};

  Serial.begin(2000000);

  printMatrix(Serial, "A", matrixA);
  printMatrix(Serial, "B", matrixB);

  printMatrix(Serial, "start", matrixAB);

  // i follows moving row on matrixA
  for (int i = 0; i < 3; i++) {
    // k follows moving column on matrixB
    for (int k = 0; k < 3; k++) {
      // j follows moving terms in matrixA and matrixB
      for (int j = 0; j < 3; j++) {

        // perform dot product
        matrixAB.arr[i][k] = matrixAB.arr[i][k] + (matrixA.arr[i][j] * matrixB.arr[j][k]);
      }
    }
  }

  printMatrix(Serial, "end", matrixAB);
  Serial.println("");
  delay(10);

  return matrixAB;
}

// multiply two 4x4 matrices

Kinematics::arrWrap4 Kinematics::multiply(struct arrWrap4 matrixA, struct arrWrap4 matrixB) {
  // declare and initialize final matrix
  // Serial.begin(2000000);
  
  struct arrWrap4 matrixAB = {{
      {0, 0, 0, 0}, 
      {0, 0, 0, 0}, 
      {0, 0, 0, 0},
      {0, 0, 0, 0}
  }};

  // printMatrix(Serial, "A", matrixA);
  // printMatrix(Serial, "B", matrixB);

  // printMatrix(Serial, "start", matrixAB);

  // i follows moving row on matrixA
  for (int i = 0; i < 4; i++) {
    // k follows moving column on matrixB
    for (int k = 0; k < 4; k++) {
      // j follows moving terms in matrixA and matrixB
      for (int j = 0; j < 4; j++) {

        // perform dot product
        matrixAB.arr[i][k] = matrixAB.arr[i][k] + (matrixA.arr[i][j] * matrixB.arr[j][k]);
      }
    }
  }

  // printMatrix(Serial, "end", matrixAB);
  // Serial.println("");
  // delay(10);

  return matrixAB;
}

// print 3x3 matrix
void Kinematics::printMatrix(HardwareSerial &serial, String title, struct arrWrap3 pose) { // pass Serial reference as argument
  delay(10); // prevents squares from appearing in serial monitor
  serial.println(title);
  
  for (int i = 0; i < 3; i++) {
    serial.println(String(pose.arr[i][0]) + " " + String(pose.arr[i][1]) + " " + String(pose.arr[i][2]));
  }
}

// print 4x4 matrix
void Kinematics::printMatrix(HardwareSerial &serial, String title, struct arrWrap4 pose) { // pass Serial reference as argument
  delay(10); // prevents squares from appearing in serial monitor
  serial.println(title);
  
  for (int i = 0; i < 4; i++) {
    serial.println(String(pose.arr[i][0]) + " " + String(pose.arr[i][1]) + " " + String(pose.arr[i][2]) + " " + String(pose.arr[i][3]));
  }
}

