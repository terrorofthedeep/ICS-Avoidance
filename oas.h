// ----- Libraries -----
#include <Arduino.h>
#include "RPC.h"
#include "NewPing.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"

////////////////////////
///       OAS        ///
////////////////////////

// ---------- Ultra-Sonic initial variable declarations ---------------
const int USRange = 300; // cm
const int maxDist = 200; // cm

// define trigger pins (Color coated wires g = green etc.)
const int trigPin_g = 23;
const int trigPin_w = 25;
const int trigPin_y = 27;
const int trigPin_o = 29;
const int trigPin_b = 31;
const int trigPin_p = 33;

// define echo pins (1 = Left : 6 = Right)
const int echoPin_1 = 22;
const int echoPin_2 = 24;
const int echoPin_3 = 26;
const int echoPin_4 = 28;
const int echoPin_5 = 30;
const int echoPin_6 = 32;

const int ARRAY_SIZE = 100;
const int MAX_SPEED = 255;
const float maxLaserDistance = 1.11;

// --------------------------------------------------------------------

// --------------------- MPU6050 variable setup -----------------------
#define T_INT .01 // 10 ms --> .01 s

void avoid_setup();

int check_IR(HardwareSerial& mySerial, unsigned char laserData[]);

void check_US();

void detectAboveObstacles(int &angle, int &speed);

void detectIngroundObstacles(int &angle, int &speed);

void check_gyro();

float getPosition(float Po, float  Vel, float Acc);

float getVelocity(float Acc, float Vo);

void trackMovement(int &angle, int &speed);

void updateYawPitchRoll(float deltaYaw, float deltaPitch, float deltaRoll);

void updateVelocity(float deltaVx, float deltaVy);

void avoid(int &angle, int &speed);
