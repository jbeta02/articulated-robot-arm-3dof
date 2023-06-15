// header file for Kinematics

#ifndef Kinematics_h
#define Kinematics_h
#include <Arduino.h>
#include <math.h>

class Kinematics {
  
  public:

    // define my datatype to make working with arrays easier
    typedef float (*arr1)[1];
    typedef float (*arr3)[3];
    typedef float (*arr4)[4];

    Kinematics();

    arr4 getPosition(float angle1, float angle2, float angle3, float link_len1, float link_len2, float base_height);

    arr1 getAngles(float link_len1, float link_len2, float base_height, float end_x, float end_y, float end_z);

    // print matrix
    void printMatrix(HardwareSerial &serial, String title, arr3 pose);
    void printMatrix(HardwareSerial &serial, String title, arr4 pose);


  private: 

    const float identityMatrix[3][3] = {
      {1, 0, 0}, 
      {0, 1, 0}, 
      {0, 0, 1}
    };

    // homogenous transformation matrix
    arr4 getHTM(arr3 rotationMatrix, arr1 displacementVector);

    // rotation be about given axis
    arr3 xRotation(float angle);

    arr3 yRotation(float angle);

    arr3 zRotation(float angle);
    

    float toRadians(float degrees);
    float toDegrees(float radians);

    // multiply two matrices
    arr3 multiply(arr3 matrixA, arr3 matrixB);
    arr4 multiply(arr4 matrixA, arr4 matrixB);

};

#endif