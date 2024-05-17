char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data_laser_1[11] = {0};
unsigned char data_laser_2[11] = {0};

void setup()
{
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial1.begin(9600);   // Initialize hardware serial port 1
  Serial2.begin(9600);   // Initialize hardware serial port 2
}

void loop(){
  checkLaser(Serial3, data_laser_1, "Left");
  checkLaser(Serial2, data_laser_2, "Right");
}


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
  //Serial.println(laserSide + "Issue");

  delay(20);
}
