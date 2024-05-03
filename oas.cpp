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

void trackMovement() {
  // angle : Max Left (60) - Max Right (120)
  // speed : 0 - 255 (Max Speed is 70 MPH
    static float angleHistory[ARRAY_SIZE];
    static float speedHistory[ARRAY_SIZE];
    static int index = 0;

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
// ===                         Set up                           ===
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
