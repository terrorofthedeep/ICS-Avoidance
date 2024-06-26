////////////////////////
///       OAS        ///
////////////////////////

// ----- Libraries -----
#include <Arduino.h>
#include "RPC.h"
#include "NewPing.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include <Servo.h>
#include "mbed.h"


//int angles[20] = {90, 90, 90, 90, 110, 90, 90, 60, 90, 90, 90, 110, 120, 90, 90, 90, 90, 90, 90, 90};
int angles[20] = {45, 45, 45, 45, 65, 65, 45, 0, 20, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45};

//---------------- variables for PID control -----------------
float kp_angle = 1.0, ki_angle = 0.05, kd_angle = 0.01;
float kp_speed = 1.0, ki_speed = 0.05, kd_speed = 0.01;
float integral_angle = 0, previous_error_angle = 0;
float integral_speed = 0, previous_error_speed = 0;

// Constants for integral windup protection
const float MAX_INTEGRAL_ANGLE = 1000.0;
const float MIN_INTEGRAL_ANGLE = -1000.0;
const float MAX_INTEGRAL_SPEED = 1000.0;
const float MIN_INTEGRAL_SPEED = -1000.0;

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

// Global Variable of actual angle the vehicle is heading and the desired of where it should be going
int actualAngle = 0;  // The direction the vehicle is acually heading 
int desiredAngle = 0; // The direction the vehicle is supposed to head
int actualSpeed = 0;
int desiredSpeed = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ---------------------------End OAS Setup-----------------------------

// ================================================================
// ===                   STAY ON PATH HARDWARE                  ===
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
    actualSpeed = sqrt(sq(Vx)+ sq(Vy));
    actualAngle = ypr[0] * 180/M_PI;

    Serial.println("actual Angle: "+ String(actualAngle));
    
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
void keepOnPath() {

    // Check the gyroscope and accelerometer to get UP TO DATE values
    check_gyro();

    // Calculate angle PID control
    float angleError = desiredAngle - actualAngle;
    integral_angle += angleError;

    Serial.println("desiredAngle: " + String(desiredAngle));
    Serial.println("actualAngle: " + String(desiredAngle));
    Serial.println("angle ErrorAngle: " + String(desiredAngle));
    Serial.println("integralAngle: " + String(desiredAngle));
    
    // Integral windup protection for angle
    if (integral_angle > MAX_INTEGRAL_ANGLE) integral_angle = MAX_INTEGRAL_ANGLE;
    if (integral_angle < MIN_INTEGRAL_ANGLE) integral_angle = MIN_INTEGRAL_ANGLE;

    float derivative_angle = angleError - previous_error_angle;
    int angleAdjustment = kp_angle * angleError + ki_angle * integral_angle + kd_angle * derivative_angle;
    previous_error_angle = angleError;

    // Calculate speed PID control
    float speedError = desiredSpeed - actualSpeed;
    integral_speed += speedError;

    // Integral windup protection for speed
    if (integral_speed > MAX_INTEGRAL_SPEED) integral_speed = MAX_INTEGRAL_SPEED;
    if (integral_speed < MIN_INTEGRAL_SPEED) integral_speed = MIN_INTEGRAL_SPEED;

    float derivative_speed = speedError - previous_error_speed;
    int speedAdjustment = kp_speed * speedError + ki_speed * integral_speed + kd_speed * derivative_speed;
    previous_error_speed = speedError;

    Serial.println("angleAdjustment: " + String(angleAdjustment));
    Serial.println("-------------------------------------------");

    vehicleAngle(angleAdjustment);
    vehicleSpeed(30); // keep at constant speed for safety

}

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
  pwmAngle->pulsewidth_us(dutyPeriod);
  delay(1);
}

void vehicleSpeed(int speed) {
  // Serial.println(speed);
  analogWrite(pinSpeed,speed);
}

// ================================================================
// ===                          Set Up                          ===
// ================================================================
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

}

// ================================================================
// ===                           Loop                           ===
// ================================================================
void loop() {
  
  desiredSpeed = 30;
  for (int i = 0; i < 20; i++){
    desiredAngle = angles[i];
    keepOnPath();
    delay(20);
  }

}

