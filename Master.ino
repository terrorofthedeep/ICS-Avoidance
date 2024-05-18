// ----- Libraries -----
#include <Arduino.h>
#include "RPC.h"
#include "NewPing.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include <Servo.h>
#include "mbed.h"

// ---------- Constants for Motor Control ----------
const int MIN_ANGLE_PWM = 1300;
const int MAX_ANGLE_PWM = 1800;
const int MIN_ANGLE = 60;
const int MAX_ANGLE = 120;
const int MIN_SPEED_PWM = 120;
const int MAX_SPEED_PWM = 1500;
PinName pinAngle = digitalPinToPinName(D2);
PinName pinSpeed = digitalPinToPinName(D4);
mbed::PwmOut* pwmAngle = new mbed::PwmOut(pinAngle);
mbed::PwmOut* pwmSpeed = new mbed::PwmOut(pinSpeed);
int speed = 0;
int angle = 90;
// --------------------------END Proc Setup ----------------------------

////////////////////////
///       OAS        ///
////////////////////////

// ---------- Ultra-Sonic initial variable declarations ---------------

const int WINDOW_SIZE = 9;
const int NUM_SENSORS = 6;
int window[NUM_SENSORS][WINDOW_SIZE];
int sortedWindow[NUM_SENSORS][WINDOW_SIZE];
int indices[NUM_SENSORS] = {0}; // Current index in the circular buffer for each sensor
bool filled[NUM_SENSORS] = {false}; // Flags to check if each sensor's window is filled


const int SENSOR_ANGLE_STEP = 30;
const int MIN_OBSTACLE_DISTANCE = 75;
const int USRange = 300; // cm
const int maxDist = 85; // cm

// define trigger pins (Color coded wires g = green etc.)
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
const int IRInterval = 500; //How often to check IR and potentially avoid obstacles

// Create sonar class attributed to each indivdual sensor
// Most left is sonar_1 and Most Right is sonar_6
NewPing sonar_1(trigPin_g, echoPin_1, USRange);
NewPing sonar_2(trigPin_w, echoPin_2, USRange);
NewPing sonar_3(trigPin_y, echoPin_3, USRange);
NewPing sonar_4(trigPin_o, echoPin_4, USRange);
NewPing sonar_5(trigPin_b, echoPin_5, USRange);
NewPing sonar_6(trigPin_p, echoPin_6, USRange);

// ---------- Infra-Red initial variable declarations -----------------
char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data_laser_1[11] = {0}; // Holds sensor data
unsigned char data_laser_2[11] = {0};
int distance_IR_1 = 0;
int distance_IR_2 = 0;
unsigned long lastIRCheckTime = 0;

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

int curr_0 = 0; // Heading of vehicle

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// --------------------------------------------------------------------

int distances[6] = {0};
int prev_distance[6] = {-1,-1,-1,-1,-1,-1};
int storedRoute[2] = {0, 0};

// ---------------------------End OAS Setup-----------------------------

// ================================================================
// ===                  STAY ON PATH FUNCTIONS                  ===
// ================================================================
void check_gyro(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // Output Yaw Pitch Roll    
    // Display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

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

    //Update Breadcrumb and send 
    currBC[0] = sqrt(sq(Vx)+ sq(Vy));
    currBC[1] = ypr[0] * 180;
    //Serial.println("YPR: " + String(ypr[0]));
    delay(10);
  }
}

float getPosition(float Po, float  Vel, float Acc){
  return Po + (Vel*T_INT) + (.5 * Acc * (T_INT * T_INT));
}

float getVelocity(float Acc, float Vo){
  return Vo + (Acc * T_INT);
}

// ================================================================
// ===                  STAY ON PATH FUNCTIONS                  ===
// ================================================================

void keepOnPath(int *desiredAngle, int *desiredSpeed) {

  // Check the gyroscope and accelerometer
  check_gyro();

  // Read current angle and speed
  int currentAngle = currBC[1]; // Assuming currBC[1] stores the current angle
  int currentSpeed = currBC[0]; // Assuming currBC[0] stores the current speed

  // Update current heading
  int newHeading = curr_0 - 90 + *desiredAngle;
  ////Serial.println("New Heading : " + String(newHeading));

  if(newHeading <= 180 && newHeading >= -180){
    curr_0 = curr_0 - 90 + *desiredAngle;
    
  }else {
    curr_0 = abs(newHeading) - 360;
  }

  Serial.println("Adjusted curr_0 : " + String(curr_0));
  Serial.println("Current Angle we are going: " + String(currentAngle));
  if(currentAngle != curr_0){

    Serial.println("Current Angle we are going: " + String(currentAngle));

    // Calculate error between desired and current values
    int angleError = curr_0 - currentAngle;
    Serial.println("Angle Error: " + String(angleError));

    if (angleError > 20){
      angleError = 20;

    }
    // Adjust the angle to turn to
    *desiredAngle = max(65, min(115, *desiredAngle + angleError));

    Serial.println("NEW/ Adjusted angle to turn to: " + String(*desiredAngle));
    Serial.println("-----------------------------");
    
  } else{
    return;
  }

}

// ================================================================
// ===                     Ultra Sonic Reroute                  ===
// ================================================================
void check_US(int* angle, int* speed){
  addValue(0, sonar_1.ping_cm());
  addValue(1, sonar_2.ping_cm());
  addValue(2, sonar_3.ping_cm());
  addValue(3, sonar_4.ping_cm());
  addValue(4, sonar_5.ping_cm());
  addValue(5, sonar_6.ping_cm());
  //Serial.print("Check_US ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (filled[i]) {
      int median = findMedian(i);

      if(median < 30){ //Emergency Stop
        *speed = 0;
        Serial.println("[EMERGENCY STOP]: Sensor ");
        Serial.print(i);
        Serial.println(" detected late avoidance ");
      }
      distances[i] = median;
      Serial.print(" Sensor ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(median);
    }
  }
  Serial.println("");
}

void addValue(int sensor, int value) {
  if(value == 0) return;
  window[sensor][indices[sensor]] = value; // Add the new value to the current index for the sensor
  indices[sensor] = (indices[sensor] + 1) % WINDOW_SIZE; // Update index in a circular manner
  if (indices[sensor] == 0) {
    filled[sensor] = true; // Set the flag when the window is filled
  }
}

int findMedian(int sensor) {
  // Copy the sensor's window values to the sortedWindow array
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sortedWindow[sensor][i] = window[sensor][i];
  }
  // Sort the sortedWindow array
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int j = i + 1; j < WINDOW_SIZE; j++) {
      if (sortedWindow[sensor][i] > sortedWindow[sensor][j]) {
        int temp = sortedWindow[sensor][i];
        sortedWindow[sensor][i] = sortedWindow[sensor][j];
        sortedWindow[sensor][j] = temp;
      }
    }
  }
  // Return the median value
  return sortedWindow[sensor][4];
}

void detectAboveObstacles(int *angle, int *speed) {
  
  // Determine which angle for the breadcrumb
  int SensorIdx[] = {0, -1, 1, -2, 2,-3, 3, -4, 4, -5, 5};
  

  
  // Determine which sensor we need to start with
  int startSens = (*angle) / 30;
  int dist = (*speed) * 2.5;
  //Serial.print("StartSens");
  ////Serial.println(startSens + 1);

  ////Serial.println("Initial Course: ");
  ////Serial.println("Angle: " + String(*angle) + " " + "Distance: " + String(dist));

  // If the starting angle is between two sensors
  if (*angle % 30 == 0){
    //Serial.print("StartSens " + String(startSens + 1) + String(startSens + 2));
    for(int i = 0; i < 11; i++){
      int sensor1 = startSens + SensorIdx[i];
      int sensor2 = startSens + SensorIdx[i] - 1;
      ////Serial.println(String(sensor1) + String(sensor2));
      if (sensor1 >= 0 && sensor1 <= 5 && sensor2 >= 0 && sensor2 <= 5 ) {
        if (distances[sensor1] > dist && distances[sensor2] > dist) {
          *angle = max(min(*angle + (SensorIdx[i]) * 30, 120), 60);
          *speed = max(20,*speed * (1 - .05 * abs(i)));
          ////Serial.println("Found New Course: ");
          ////Serial.println("Sensor: "+ String(startSens + SensorIdx[i] + 1) + "Angle: " + String(*angle) + " " + "Speed: " + String(*speed));
          return;
        }
      }
    }
  }
  //If the starting angle is not
  else{
    //Serial.print("StartSens" + String(startSens + 1) );
    for(int i = 0; i < 11; i++){
      if ((startSens + SensorIdx[i] >= 0) && (startSens + SensorIdx[i] <= 5)) {
        ////Serial.println(distances[startSens + SensorIdx[i]]);
        if (distances[startSens + SensorIdx[i]] > dist) {
          *angle = max(min(*angle + (SensorIdx[i]) * 30, 120), 60);
          *speed = min(20, *speed * (1 - .05 * abs(i)));
          ////Serial.println("Found New Course: ");
          ////Serial.println("Sensor: "+ String(startSens + SensorIdx[i] + 1) + " Angle: " + String(*angle) + " " + "Speed: " + String(*speed));
          return;
        }
      }
    }
  }
  ////Serial.println("No valid avoid found");
  *speed = 0;
  return;

}

// ================================================================
// ===                         Avoid                            ===
// ================================================================

void avoid(int *angle, int *speed) {

  //Checks the sensors
  check_US(angle, speed);

  //Check to see if newRoute is a valid route
  detectAboveObstacles(angle, speed);
  
  //Uses the gyroscope to calculate any micro-adjustments we need
  keepOnPath(angle, speed);

}

// -----------------------End OAS Functions------------------------

// ================================================================
// ===                      Motor Control                       ===
// ================================================================
void vehicleAngle(int angle) {
  float scale = float(float(MAX_ANGLE_PWM - MIN_ANGLE_PWM) / float(MAX_ANGLE - MIN_ANGLE));
  float offset = -MIN_ANGLE * scale + MIN_ANGLE_PWM;
  if (angle < 60) {
    angle = 60;
  } else if (angle > 120) {
    angle = 120;
  }
  int dutyPeriod = int(scale * angle) + offset;
  Serial.println("Duty period: " + String(dutyPeriod));
  pwmAngle->pulsewidth_us(dutyPeriod);
  delay(1);
}

void vehicleSpeed(int speed) {
  // Serial.println(speed);
  analogWrite(pinSpeed,speed);
}

///////////
///Setup///
///////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  Serial.begin(115200);

  // initialize device
  mpu.initialize();
  
  // verify connection
  ////Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
      //Serial.print(F("DMP Initialization failed (code "));
  }

  for (int i = 0; i < 6; i++) {
      for(int j = 0; j < WINDOW_SIZE; j++){
        window[i][j] = 0;
      }
    }


}

//////////
///LOOP///
//////////
void loop() {

  speed = 50;
  angle = 90;
  int *speedPtr = &speed;
  int *anglePtr = &angle;

  avoid(anglePtr, speedPtr);
  vehicleAngle(*anglePtr);
  vehicleSpeed(*speedPtr);

}

