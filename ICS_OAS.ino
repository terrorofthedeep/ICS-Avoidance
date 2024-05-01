// ----- Libraries -----
#include <Arduino.h>
#include "RPC.h"
#include "NewPing.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ----- Global Var-----

// Infra-Red initial variable declarations
char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data_laser_1[11] = {0}; // Holds sensor data
unsigned char data_laser_2[11] = {0};
// ----------------------------------

// Ultra-Sonic initial variable declarations
const int frontMaxDist = 300; // cm
const int sideMaxDist = 30; // cm

// define trigger pins (1 = Left : 6 = Right)
const int trigPin_1 = 23;
const int trigPin_2 = 25;
const int trigPin_3 = 27;
const int trigPin_4 = 29;
const int trigPin_5 = 31;
const int trigPin_6 = 33;

// define echo pins (Color coated wires g = green etc.)
const int echoPin_g = 22;
const int echoPin_w = 24;
const int echoPin_y = 26;
const int echoPin_o = 28;
const int echoPin_b = 30;
const int echoPin_p = 32;

// Create sonar class attributed to each indivdual sensor
// Most left is sonar_1 and Most Right is sonar_6
NewPing sonar_1(trigPin_1, echoPin_g, frontMaxDist);
NewPing sonar_2(trigPin_2, echoPin_w, frontMaxDist);
NewPing sonar_3(trigPin_3, echoPin_y, frontMaxDist);
NewPing sonar_4(trigPin_4, echoPin_o, frontMaxDist);
NewPing sonar_5(trigPin_5, echoPin_b, frontMaxDist);
NewPing sonar_6(trigPin_6, echoPin_p, frontMaxDist);

// Array to store US data
int distance_US[6] = {0};

// ----------------------------------

// MPU6050 variable setup
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
float breadcrumb[2];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ----------------------------------

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    
    //exit(0);
    mpu.initialize();
    
    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(81);
    mpu.setYGyroOffset(-9);
    mpu.setZGyroOffset(52);
    mpu.setZAccelOffset(1439); // 1688 factory default for my test chip
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  //Serial.begin(9600); // Starts the serial communication
  Serial1.begin(9600);   // Initialize hardware serial port 1 (IR)
  Serial2.begin(9600);   // Initialize hardware serial port 2 (IR)
}

void loop(){

  check_US(distance_US);                          // Check all 6 US sensors
  checkLaser(Serial1, data_laser_1, "Left");  // Check Left IR Laser
  checkLaser(Serial2, data_laser_2, "Right"); // Check Right IR Laser
  check_gyro();
}

/* 
Purpose: To read and write to IR sensors for detecting inground obstacles
Input:  mySerial : needs to be a hardware serial port defined by Giga
        laserData: specific array assigned to the laser
        laserSide: string stating what side the laser is on
Output: VOID
*/
void checkLaser(HardwareSerial& mySerial, unsigned char laserData[], String laserSide){

  delay(10);
  mySerial.write(buff, 4); // Send data to sensor 1
  //Serial2.write(buff, 4); // Send data to sensor 2

  
  if (mySerial.available() > 0) // Determine whether there is data to read on serial 1
  {
    delay(50);
    for (int i = 0; i < 11; i++)
    {
      laserData[i] = mySerial.read();
    }

    unsigned char Check = 0;
    for (int i = 0; i < 10; i++)
    {
      Check = Check + laserData[i];
    }
    Check = ~Check + 1;
    if (laserData[10] == Check)
    {
      if (laserData[3] == 'E' && laserData[4] == 'R' && laserData[5] == 'R')
      {
        Serial.println(laserSide + "Sensor Out of range");
      }
      else
      {
        float distance = 0;
        distance = (laserData[3] - 0x30) * 100 + (laserData[4] - 0x30) * 10 + (laserData[5] - 0x30) * 1 + (laserData[7] - 0x30) * 0.1 + (laserData[8] - 0x30) * 0.01 + (laserData[9] - 0x30) * 0.001;
        Serial.print(laserSide + "Sensor - Distance = ");
        Serial.print(distance, 3);
        Serial.println(" M");
      }
    }
    else
    {
      Serial.println(laserSide + " Sensor - Invalid Data!");
    }
  }

  delay(20);
}

/* 
Purpose: to get all the distances from the US sensors
Input:  distances : int array of the distances from respective US sensor
Output: VOID
*/
void check_US(int distances[]){
  distances[0] = sonar_1.ping_cm(); // Store distance from sonar 1
  Serial.println(sonar_1.ping_cm());
  distances[1] = sonar_2.ping_cm(); // Store distance from sonar 2
  distances[2] = sonar_3.ping_cm(); // Store distance from sonar 3
  distances[3] = sonar_4.ping_cm(); // Store distance from sonar 4
  distances[4] = sonar_5.ping_cm(); // Store distance from sonar 5
  distances[5] = sonar_6.ping_cm(); // Store distance from sonar 6
  for(int i = 0; i < 6; i++) {
    Serial.print("Distance ");
    Serial.print(i);
    Serial.print(":");
    Serial.print(distances[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

/* 
Purpose: to get speed and angle 
Input:  
Output: VOID
*/
void check_gyro(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // Output Yaw Pitch Roll    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // YAW
    Serial.print("Yaw:");
    Serial.print(ypr[0] * 180/M_PI);
    // PITCH     
    Serial.print(", Pitch: ");  
    Serial.print(ypr[1] * 180/M_PI);
    // ROLL        
    Serial.print(", Roll: ");
    Serial.println(ypr[2] * 180/M_PI);


    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // X
    Serial.print("X Accel:");
    Serial.print(aaReal.x/ 16384.0);
    // Y
    Serial.print(", Y Accel:");
    Serial.print(aaReal.y/ 16384.0);
    // Z
    Serial.print(", Z Accel:");
    Serial.println(aaReal.z/ 16384.0);

    ax = (aaReal.x/ 16384.0) * 9.8;
    ay = (aaReal.y/ 16384.0) * 9.8;
    Vx = getVelocity(Vo_x, ax);
    Vy = getVelocity(Vo_y, ay);
        
    Px = getPosition(Po_x, Vx, ax);
    Py = getPosition(Po_y, Vy, ay);
        
    //Update initial values 
    Vo_x = Vx;
    Vo_y = Vy;
    Po_x = Px;
    Po_y = Py;

    //Pack breadcrumb and send 
    breadcrumb[0] = sqrt(sq(Vx)+ sq(Vy));
    breadcrumb[1] = ypr[0];
        
    delay(10);
  }
}

// ================================================================
// ===               POSITION MODULE FUNCTIONS                  ===
// ================================================================

float getPosition(float Po, float  Vel, float Acc){
  return Po + (Vel*T_INT) + (.5 * Acc * (T_INT * T_INT));
}

float getVelocity(float Acc, float Vo){
  return Vo + (Acc * T_INT);
}
