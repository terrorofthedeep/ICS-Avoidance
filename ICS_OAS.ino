// ----- Libraries -----
#include "NewPing.h"

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

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial1.begin(9600);   // Initialize hardware serial port 1 (IR)
  Serial2.begin(9600);   // Initialize hardware serial port 2 (IR)
}

void loop(){
  check_US(distance_US);                          // Check all 6 US sensors
  checkLaser(Serial1, data_laser_1, "Left");  // Check Left IR Laser
  checkLaser(Serial2, data_laser_2, "Right"); // Check Right IR Laser
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
  distances[1] = sonar_2.ping_cm(); // Store distance from sonar 2
  distances[2] = sonar_3.ping_cm(); // Store distance from sonar 3
  distances[3] = sonar_4.ping_cm(); // Store distance from sonar 4
  distances[4] = sonar_5.ping_cm(); // Store distance from sonar 5
  distances[5] = sonar_6.ping_cm(); // Store distance from sonar 6
}
