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

// ---------- Infra-Red initial variable declarations -----------------
char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data_laser_1[11] = {0}; // Holds sensor data
unsigned char data_laser_2[11] = {0};
// --------------------------------------------------------------------

// ---------- Ultra-Sonic initial variable declarations ---------------
const int frontMaxDist = 300; // cm
const int sideMaxDist = 30; // cm

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

// Create sonar class attributed to each indivdual sensor
// Most left is sonar_1 and Most Right is sonar_6
NewPing sonar_1(trigPin_g, echoPin_1, sideMaxDist);
NewPing sonar_2(trigPin_w, echoPin_2, frontMaxDist);
NewPing sonar_3(trigPin_y, echoPin_3, frontMaxDist);
NewPing sonar_4(trigPin_o, echoPin_4, frontMaxDist);
NewPing sonar_5(trigPin_b, echoPin_5, frontMaxDist);
NewPing sonar_6(trigPin_p, echoPin_6, sideMaxDist);
// --------------------------------------------------------------------

// --------------------- MPU6050 variable setup -----------------------
#define T_INT .01 // 10 ms --> .01 s
MPU6050 mpu;      // MPU 6050 object

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// other Variables
float ax = 0;
float ay = 0;
float Vx, Vo_x = 0;
float Vy, Vo_y = 0;
float Px, Po_x = 0;
float Py, Po_y = 0;
float currBC[2] = {0};

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// --------------------------------------------------------------------

void setup();

//void checkLaser(HardwareSerial& mySerial, unsigned char laserData[], String laserSide);

//void check_US();

void check_gyro();

float getPosition(float Po, float  Vel, float Acc);

float getVelocity(float Acc, float Vo);

void trackMovement();

void updateYawPitchRoll(float deltaYaw, float deltaPitch, float deltaRoll)

void updateVelocity(float deltaVx, float deltaVy)
