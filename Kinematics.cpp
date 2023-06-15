// source file for Kinematics

#include "Kinematics.h"
#include <Arduino.h>
#include <math.h>


Kinematics::Kinematics() {

}


Kinematics::arr4 Kinematics::getPosition(float angle1, float angle2, float angle3, float link_len1, float link_len2, float base_height) {
  // convert angles from dgrees to radians
  angle1 = toRadians(angle1);
  angle2 = toRadians(angle2);
  angle3 = toRadians(angle3);


  // projection of coordiante from 1 on coordinate frame 0
  const float projection1_on_0[3][3] = {
    {1, 0, 0}, 
    {0, 0, -1}, 
    {0, 1, 0}
  };


  // rotation matrices
  arr3 R_01 = multiply(projection1_on_0, yRotation(angle1));
  arr3 R_12 = multiply(identityMatrix, zRotation(angle2));
  arr3 R_23 = multiply(identityMatrix, zRotation(angle3));

  // displacement vectors
  float d_01[3][1] = {{0}, {0}, {base_height}};
  float d_12[3][1] = {{link_len1 * cos(angle2)}, {link_len2 * sin(angle2)}, {0}};
  float d_23[3][1] = {{link_len2 * cos(angle3)}, {link_len2 * sin(angle3)}, {0}};

  // homogenous transformation matrices
  arr4 htm_01 = getHTM(R_01, d_01);
  arr4 htm_12 = getHTM(R_12, d_12);
  arr4 htm_23 = getHTM(R_23, d_23);

  //arr4 htm_03  = multiply(multiply(htm_01, htm_12), htm_23);

  float matrix1[4][4] = {
          {1, 0, 0, 0}, 
          {0, 1, 0, 0}, 
          {0, 0, 1, 0},
          {0, 0, 0, 1}
  };

  float matrix2[4][4] = {
          {12, 22, 12, 12}, 
          {42, 52, 62, 12}, 
          {22, 32, 52, 12},
          {22, 32, 52, 12}
  };

  float matrix4[4][4] = {
          {2, 2, 2, 2}, 
          {4, 5, 6, 1}, 
          {2, 3, 5, 1},
          {2, 3, 5, 1}
  };

  arr4 matrix3 = multiply(matrix1, matrix2);

  Serial.begin(2000000);
  printMatrix(Serial, "matrix3", matrix3);

  arr4 matrix8 = multiply(matrix1, matrix2);

  printMatrix(Serial, "matrix8", matrix8);

  //arr4 matrix5 = multiply(matrix1, matrix4);


  return htm_23;
}

// implement inverse kinematics using analytical approach by graphical method and trig
Kinematics::arr1 Kinematics::getAngles(float link_len1, float link_len2, float base_height, float end_x, float end_y, float end_z) {
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

  float angles[3][1] = {{angle1}, {angle2}, {angle3}};

  return angles;
}

// homogenous transformation matrix
Kinematics::arr4 Kinematics::getHTM(arr3 rotationMatrix, arr1 displacementVector) {
  static float htm[4][4] = {
    {rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], displacementVector[0][0]}, 
    {rotationMatrix[0][0], rotationMatrix[0][0], rotationMatrix[0][0], displacementVector[0][1]}, 
    {rotationMatrix[0][0], rotationMatrix[0][0], rotationMatrix[0][0], displacementVector[0][2]},
    {0, 0, 0, 1}
  };

  return htm;
}


// rotation be about given axis
Kinematics::arr3 Kinematics::xRotation(float angle) {
  static float xRotationMatrix[3][3] = {
    {1, 0, 0}, 
    {0, cos(angle), -sin(angle)}, 
    {0, sin(angle), cos(angle)}
  };

  return xRotationMatrix;
}

Kinematics::arr3 Kinematics::yRotation(float angle) {  
  static float yRotationMatrix[3][3] = {
    {cos(angle), 0, sin(angle)}, 
    {0, 1, 0}, 
    {-sin(angle), 1, cos(angle)}
  };

  return yRotationMatrix;
}

Kinematics::arr3 Kinematics::zRotation(float angle) {
  static float zRotationMatrix[3][3] = {
    {cos(angle), -sin(angle), 0}, 
    {sin(angle), cos(angle), 0}, 
    {0, 0, 1}
  };

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
Kinematics::arr3 Kinematics::multiply(arr3 matrixA, arr3 matrixB) {
  // declare and initialize final matrix
  static float matrixAB[3][3] = {
          {0, 0, 0}, 
          {0, 0, 0}, 
          {0, 0, 0}
  };

  // i follows moving row on matrixA
  for (int i = 0; i < 3; i++) {
    // k follows moving column on matrixB
    for (int k = 0; k < 3; k++) {
      // j follows moving terms in matrixA and matrixB
      for (int j = 0; j < 3; j++) {

        // perform dot product
        matrixAB[i][k] = matrixAB[i][k] + (matrixA[i][j] * matrixB[j][k]);
      }
    }
  }

  return matrixAB;
}

// multiply two 4x4 matrices

Kinematics::arr4 Kinematics::multiply(arr4 matrixA, arr4 matrixB) {
  // declare and initialize final matrix
  Serial.begin(2000000);
  static float matrixAB[4][4] = { // this array keeps getting saved and overriten with each funtion call (bc it is static)
          {0, 0, 0, 0}, 
          {0, 0, 0, 0}, 
          {0, 0, 0, 0},
          {0, 0, 0, 0}
  };

  printMatrix(Serial, "A", matrixA);
  printMatrix(Serial, "B", matrixB);

  printMatrix(Serial, "start", matrixAB);

  // i follows moving row on matrixA
  for (int i = 0; i < 4; i++) {
    // k follows moving column on matrixB
    for (int k = 0; k < 4; k++) {
      // j follows moving terms in matrixA and matrixB
      for (int j = 0; j < 4; j++) {

        // perform dot product
        matrixAB[i][k] = matrixAB[i][k] + (matrixA[i][j] * matrixB[j][k]);
      }
    }
  }

  printMatrix(Serial, "end", matrixAB);
  Serial.println("");
  delay(10);

  return matrixAB;
}

// print 3x3 matrix
void Kinematics::printMatrix(HardwareSerial &serial, String title, arr3 pose) { // pass Serial reference as argument
  delay(10); // prevents squares from appearing in serial monitor
  serial.println(title);
  
  for (int i = 0; i < 3; i++) {
    serial.println(String(pose[i][0]) + " " + String(pose[i][1]) + " " + String(pose[i][2]));
  }
}

// print 4x4 matrix
void Kinematics::printMatrix(HardwareSerial &serial, String title, arr4 pose) { // pass Serial reference as argument
  delay(10); // prevents squares from appearing in serial monitor
  serial.println(title);
  
  for (int i = 0; i < 4; i++) {
    serial.println(String(pose[i][0]) + " " + String(pose[i][1]) + " " + String(pose[i][2]) + " " + String(pose[i][3]));
  }
}

