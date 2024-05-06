#include "oas.h"

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

int distances[6] = {0};
int breadcrumb[2] = {0};

// Create sonar class attributed to each indivdual sensor
// Most left is sonar_1 and Most Right is sonar_6
NewPing sonar_1(trigPin_g, echoPin_1, USRange);
NewPing sonar_2(trigPin_w, echoPin_2, USRange);
NewPing sonar_3(trigPin_y, echoPin_3, USRange);
NewPing sonar_4(trigPin_o, echoPin_4, USRange);
NewPing sonar_5(trigPin_b, echoPin_5, USRange);
NewPing sonar_6(trigPin_p, echoPin_6, USRange);

// Array to store US data
int distance_US[6] = {0};

// ---------- Infra-Red initial variable declarations -----------------
char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data_laser_1[11] = {0}; // Holds sensor data
unsigned char data_laser_2[11] = {0};
int distance_IR_1 = 0;
int distance_IR_2 = 0;
// --------------------------------------------------------------------

// ================================================================
// ===               POSITION MODULE FUNCTIONS                  ===
// ================================================================

void check_gyro(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // OUtput Yaw Pitch Roll    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // YAW
    //Serial.print(ypr[0] * 180/M_PI);
    // pTICh       
    //Serial.print(ypr[1] * 180/M_PI);
    // ROLL        
    //Serial.println(ypr[2] * 180/M_PI);


    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // X
    //Serial.print(aaReal.x/ 16384.0);
    // Y
    //Serial.print(aaReal.y/ 16384.0);
    // Z
    //Serial.println(aaReal.z/ 16384.0);

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

    //Pack C and send 
    currBC[0] = sqrt(sq(Vx)+ sq(Vy));
    currBC[1] = ypr[0];
        
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

void trackMovement(int &angle, int &speed) {
  // angle : Max Left (60) - Max Right (120)
  // speed : 0 - 255 (Max Speed is 70 MPH
  static float angleHistory[ARRAY_SIZE];
  static float speedHistory[ARRAY_SIZE];
  static int index = 0;

  check_gyro();

  // Store the current angle and speed in the local arrays
  angleHistory[index] = ypr[0];
  speedHistory[index] = currBC[0];
  index = (index + 1) % ARRAY_SIZE; // Wrap around the array

  // Compare the actual movement (from simulated values) with the expected movement (from angle and speed)
  //float expectedDistance = speedHistory[index] * 1.0; // Assuming a time interval of 1 second
  //float actualDistance = sqrt(sq(Vx) + sq(Vy));
  float expectedDistance = speedHistory[index] * T_INT;
  float actualDistance = currBC[0] * T_INT; // Use the current speed (currBC[0]) as the actual distance traveled
  float distanceError = actualDistance - expectedDistance;

  float expectedAngle = angleHistory[index];
  //float actualAngle = ypr[0];
  float actualAngle = ypr[0] * 180.0 / M_PI; // Convert ypr[0] from radians to degrees
  float angleError = actualAngle - expectedAngle;

  // Course correction using vector addition of angle error and new desired angle
  float newDesiredAngle = expectedAngle + angleError;

  // Normalize the new desired angle to the range [60, 120] degrees
  newDesiredAngle = fmod(newDesiredAngle, M_PI); // Wrap to [0, PI]
  if (newDesiredAngle < M_PI / 3.0) {
    newDesiredAngle += 2.0 * M_PI / 3.0; // Shift to [60, 120] range
  }

  // Set the new desired angle and speed
  currBC[1] = newDesiredAngle;
  currBC[0] = speedHistory[index]; // Maintain the same speed for now

  // Change the global angle and speed
  angle = currBC[1] * 180.0 / M_PI; // convert to degrees
  speed = currBC[0];

  // Print the updated values for verification
  Serial.print("Angle (deg): ");
  Serial.print(currBC[1] * 180.0 / M_PI);
  Serial.print(" Speed: ");
  Serial.println(currBC[0]);
}

void updateYawPitchRoll(float deltaYaw, float deltaPitch, float deltaRoll) {
    ypr[0] += deltaYaw;
    ypr[1] += deltaPitch;
    ypr[2] += deltaRoll;
}

void updateVelocity(float deltaVx, float deltaVy) {
    Vx += deltaVx;
    Vy += deltaVy;

    // Limit the speed to the maximum value
    float speed = sqrt(sq(Vx) + sq(Vy));
    if (speed > MAX_SPEED) {
        Vx = (Vx / speed) * MAX_SPEED;
        Vy = (Vy / speed) * MAX_SPEED;
    }
}

// ================================================================
// ===                     Ultra Sonic Reroute                  ===
// ================================================================
void check_US(){
  distances[0] = sonar_1.ping_cm(); // Store distance from sonar 1
  distances[1] = sonar_2.ping_cm(); // Store distance from sonar 2
  distances[2] = sonar_3.ping_cm(); // Store distance from sonar 3
  distances[3] = sonar_4.ping_cm(); // Store distance from sonar 4
  distances[4] = sonar_5.ping_cm(); // Store distance from sonar 5
  distances[5] = sonar_6.ping_cm(); // Store distance from sonar 6
}

void detectAboveObstacles(int &angle, int &speed) {
  // Determine which angle for the breadcrumb
  int Direction[] = {0, -5, 5, -10, 10, -15, 15, -20, 20, -30, 30};
  int SensorIdx[] = {0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5};

  // Determine which sensor we need to start with
  int startSens = (Angle) / 30;
  //Serial.print("StartSens");
  //Serial.println(startSens + 1);
  for (int i = 0; i < 11; i++) {
    if (angle % 30 == 0 && i == 0 && angle < 150 && angle > -150) {
      //If the angle is in between 2 sensors, check both
      if (distance_US[startSens] > maxDist && distance_US[startSens - 1] > maxDist) {
        //result[0] = angle;
        //result[2] = speed;
        return;
      }
    }
    else if ((startSens + SensorIdx[i] >= 0) && (startSens + SensorIdx[i] < 5)) {
      if (distance_US[startSens + SensorIdx[i]] > maxDist) {
        angle += Direction[i];
        //Reduce speed by 5% per sensor 
        speed *= (1 - (abs(SensorIdx) * .05));
        return;
      }
    }
  }
  speed = 0;

}

// ================================================================
// ===                      Infrared Reroute                    ===
// ================================================================
int check_IR(HardwareSerial& mySerial, unsigned char laserData[]) {
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
        //Serial.println(laserSide + "Sensor Out of range");
        // distance_IR = -1; // Not valid
        return -1;
      }
      else
      {
        //float distance = 0;
        // distance_IR = (laserData[3] - 0x30) * 100 + (laserData[4] - 0x30) * 10 + (laserData[5] - 0x30) * 1 + (laserData[7] - 0x30) * 0.1 + (laserData[8] - 0x30) * 0.01 + (laserData[9] - 0x30) * 0.001;
        return (laserData[3] - 0x30) * 100 + (laserData[4] - 0x30) * 10 + (laserData[5] - 0x30) * 1 + (laserData[7] - 0x30) * 0.1 + (laserData[8] - 0x30) * 0.01 + (laserData[9] - 0x30) * 0.001;
        //Serial.print(laserSide + "Sensor - Distance = ");
        //Serial.print(distance, 3);
        //Serial.println(" M");
      }
    }
    else
    {
      // Serial.println(laserSide + " Sensor - Invalid Data!");
    }
  }

  delay(20);
}

void detectIngroundObstacles(int &angle, int &speed) {
  //1 = left 2 = right
  // Check if left and right are both valid
  // 
  if(distance_IR_1 > maxLaserDistance && distance_IR_2 > maxLaserDistance) {
    // Valid path
    return; 
  }
  else if(distance_IR_1 > maxLaserDistance && distance_IR_2 <= maxLaserDistance){
    // Turn left
    angle = 75;
    speed *= 0.75;
  }
  else if(distance_IR_1 <= maxLaserDistance && distance_IR_2 > maxLaserDistance) {
    // Turn right
    angle = 105;
    speed = 0.75;

  } else {
    // stop
    speed = 0;
  }

}
// ================================================================
// ===                         Avoid                            ===
// ================================================================

void avoid(int &angle, int &speed) {
  check_gyro();
  trackMovement(angle, speed);

  distance_IR_1 = check_IR(Serial1, data_laser_1);
  distance_IR_2 = check_IR(Serial2, data_laser_2);
  detectIngroundObstacles(angle, speed);

  check_US();
  detectAboveObstacles(angle, speed);
  
  
}

// ================================================================
// ===                         Set up                           ===
// ================================================================

void avoid_setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
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
