#include "oas.h"

#define WINDOW_SIZE 9
#define NUM_SENSORS 6
#define CRIT_DIST 90
MPU6050 mpu;      // MPU 6050 object
//SerialPort serialPort;

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
int curr_0 = 0;


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

int window[NUM_SENSORS][WINDOW_SIZE];
int sortedWindow[NUM_SENSORS][WINDOW_SIZE];
int indices[NUM_SENSORS] = {0}; // Current index in the circular buffer for each sensor
bool filled[NUM_SENSORS] = {false}; // Flags to check if each sensor's window is filled

int avoidstack[3] = {0};
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
// --------------------------------------------------------------------

// ================================================================
// ===               POSITION MODULE FUNCTIONS                  ===
// ================================================================

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

  //Serial.println("Adjusted curr_0 : " + String(curr_0));
  //Serial.println("Current Angle we are going: " + String(currentAngle));
  if(currentAngle != curr_0){

    //Serial.println("Current Angle we are going: " + String(currentAngle));

    // Calculate error between desired and current values
    int angleError = curr_0 - currentAngle;
    //Serial.println("Angle Error: " + String(angleError));

    if (angleError > 20){
      angleError = 20;

    }
    // Adjust the angle to turn to
    *desiredAngle = max(65, min(115, *desiredAngle + angleError));

    //Serial.println("NEW/ Adjusted angle to turn to: " + String(*desiredAngle));
    //Serial.println("-----------------------------");
    
  } else{
    return;
  }

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
}// ================================================================
// ===                  GET BACK ON PATH FUNCTIONS                  ===
// ================================================================
void getNewVector(int *angle, int *speed) {
    // Get the old x and y vectors
    int storedAngle = storedRoute[0];
    int storedSpeed = storedRoute[1];
    
    //If we don't have a storedRoute {0,0}, just return
    if (storedSpeed == 0) {
      return;
    }

    // Convert stored vector to Cartesian coordinates
    double xCoord = cos(storedAngle * M_PI / 180.0) * storedSpeed;
    double yCoord = sin(storedAngle * M_PI / 180.0) * storedSpeed;
    
    
    // Get the new x and y vectors
    double xNewCoord = cos(*angle * M_PI / 180.0) * *speed;
    double yNewCoord = sin(*angle * M_PI / 180.0) * *speed;
    
    
    // Combine the vectors
    xCoord += xNewCoord;
    yCoord += yNewCoord;
    
    
    // Calculate the new angle and speed
    int newSpeed = *speed;
    double newAngle = atan2(yCoord, xCoord) * 180.0 / M_PI;
    
    // Make sure the angle is positive
    if (newAngle < 0) {
        newAngle += 360;
    }
    
    // Update the angle and speed pointers
    *angle = (int)newAngle;
    *speed = (int)newSpeed;
}


// ================================================================
// ===                     Ultra Sonic Reroute                  ===
// ================================================================
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

      
      distances[i] = median;
      Serial.print(" Sensor ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(median);
      if(checkTurn(angle, speed, i, median)){
        break;
      }
    }
  }
  Serial.println("");
}
bool checkTurn(int* angle, int* speed, int sensor, int distance){
  if(distance < 30){ //Emergency Stop
        *speed = 0;
        Serial.println("[EMERGENCY STOP]: Sensor ");
        Serial.print(sensor+1);
        Serial.println(" detected late avoidance ");
      }
  if(*angle < 90){
    if(sensor > 2) return false;
    if(distance < CRIT_DIST){ //if obstacle detected on left
      *speed = 0;
      *angle = 120; //then turn right
      return true;
    }
  }
  else{
    if(*angle > 90){
      if(sensor < 2) return false;
      if(distance < CRIT_DIST){ //if obstacle detected on right
        *speed = 0;
        *angle = 60; //then turn left
        return true;
      }
    }if(sensor > 0 && sensor < 5){
      if(distance < CRIT_DIST){
        if(*speed > 50) *speed = *speed/2;
        Serial.print("[OBSTACLE AT SENSOR");
        Serial.print(sensor+1);
        Serial.print(" ATTEMPTING TURN ] -->");
        if(sensor <= 2){
          *angle = 119;
          Serial.print(*angle);
        } 
        else{
          *angle = 60;
          Serial.print("60");
        }
        Serial.println("");
        return true;
      }
    }
    /*
    if(sensor > 0 && sensor < 5){ //check all sensors ahead
      if(distance < CRIT_DIST){ //if obstacle detected ahead
        Serial.print("[OBSTACLE AT SENSOR");
        Serial.print(sensor+1);
        Serial.print(" ATTEMPTING TURN ] -->");
        
        int open_sensor = 0;
        int greatest = -1;
        int new_angle = 0;
        for(int i = 0; i < NUM_SENSORS; i++){ //find most open sensor
          if(i+1 != sensor){
            if(distances[i] > greatest) 
              open_sensor = i;
              greatest = distances[i];
          }
        }
        //turn towards open sensor
        if(open_sensor <= 2 ){
          new_angle = 60;//90 - ((open_sensor+1)*15);
        }
        else{
          new_angle = 120;//95 + 15*(open_sensor % 3);
        }
        Serial.print(new_angle);
        Serial.println(""); 
        *angle = new_angle;
        *speed = *speed/2;
        return true;
        }*/
    return false;

  }
}

/*
void detectAboveObstacles(int *angle, int *speed) {
  
  // Determine which angle for the breadcrumb
  int SensorIdx[] = {0, -1, 1, -2, 2,-3, 3, -4, 4, -5, 5};
  

  
  // Determine which sensor we need to start with
  int startSens = (*angle) / 30;
  int dist = (*speed) * 2.5;
  int offset = -10;
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
          Serial.println("Sensor: "+ String(startSens + SensorIdx[i] + 1) + " Angle: " + String(*angle) + " " + "Speed: " + String(*speed));
          return;
        }
      }
    }
  }
  ////Serial.println("No valid avoid found");
  *speed = 0;
  return;

}
*/
// ================================================================
// ===                      Infrared Reroute                    ===
// ================================================================
//Gets the sensor distance from mySerial if possible, otherwise returns -1
int check_IR(HardwareSerial& mySerial, unsigned char laserData[]) {
  // Clear the receive buffer
  while (mySerial.available() > 0) {
    mySerial.read();
  }
  
  delay(10);
  
  mySerial.write(buff, 4); // Send data to sensor 1

  
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
        ////Serial.println(laserSide + "Sensor Out of range");
        return -1;
      }
      else
      {
        int distance_IR = 0;
        distance_IR = ((laserData[3] - 0x30) * 100 + (laserData[4] - 0x30) * 10 + (laserData[5] - 0x30) * 1 + (laserData[7] - 0x30) * 0.1 + (laserData[8] - 0x30) * 0.01 + (laserData[9] - 0x30) * 0.001);

        //Serial.print("Sensor - Distance = ");
        //Serial.print(distance, 3);
        ////Serial.println(" M");
        return distance_IR;

      }
    }
    else
    {
      ////Serial.println("Sensor - Invalid Data!");
      return -1;
    }
  }
  else {
    ////Serial.println("Sensor unavailable!");
    return -1;
  }
}

void detectIngroundObstacles(int* angle, int* speed) {
  //1 = left 2 = right
  // Check if left and right are both valid
  // 
  if(distance_IR_1 > maxLaserDistance && distance_IR_2 > maxLaserDistance) {
    // Valid path
    return; 
  }
  else if(distance_IR_1 > maxLaserDistance && distance_IR_2 <= maxLaserDistance){
    // Turn left
    *angle = 70;
    *speed *= 0.75;
  }
  else if(distance_IR_1 <= maxLaserDistance && distance_IR_2 > maxLaserDistance) {
    // Turn right
    *angle = 110;
    *speed *= 0.75;

  } else {
    // stop
    *speed = 0;

  }

}

// ================================================================
// ===                         Avoid                            ===
// ================================================================

void avoid(int *angle, int *speed) {

  //Serial.println("Entering avoid");

  
  //Checks the sensors
  check_US(angle, speed);
  //distance_IR_1 = check_IR(Serial1, data_laser_1);
  //distance_IR_2 = check_IR(Serial2, data_laser_2);

  //Makes sure our angle is accurate compared to what our gyro angle says
  //Serial.println("StoredRoute:" + String(storedRoute[0]) + " " + String(storedRoute[1]));
  //Combine our givenRoute with our storedRoute into newRoute
  //getNewVector(angle, speed);
  //Serial.println("CombinedRoute:" + String(*angle) + " " + String(*speed));

  //Store the angle given by 
  int originalAngle = *angle;
  int originalSpeed = *speed;

  //Check to see if newRoute is a valid route
  //detectAboveObstacles(angle, speed);
  //detectIngroundObstacles(angle, speed);
  //Serial.println("USRoute:" + String(*angle) + " " + String(*speed));
  
  

  //If it is not, then storedRoute = newRoute + storedRoute, give alternatePath
  // //If it is, then storedRoute = {0,0}, give storedRoute
  // if (*angle != originalAngle){
  //   //generates the 
  //   //Serial.println("Storing old speed and distance data");
  //   storedRoute[0] = 90 - (*angle - 90);
  //   storedRoute[1] = originalSpeed;
  //   //Serial.println("StoredRoute: " + String(storedRoute[0]) + " " + String(storedRoute[1]));
  // }else{
  //   storedRoute[0] = 0;
  //   storedRoute[1] = 0;
  // }
  
  //Uses the gyroscope to calculate any micro-adjustments we need
  //keepOnPath(angle, speed);

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
    //serialPort.begin(115200);
    //delay(5000);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

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
    for (int i = 0; i < 6; i++) {
      for(int j = 0; j < WINDOW_SIZE; j++){
        window[i][j] = 0;
      }
    }
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
        //Serial.print(devStatus);
        ////Serial.println(F(")"));
    }

  //Serial.begin(9600); // Starts the serial communication
  //Serial1.begin(9600);   // Initialize hardware serial port 1 (IR)
  //Serial2.begin(9600);   // Initialize hardware serial port 2 (IR)

}
