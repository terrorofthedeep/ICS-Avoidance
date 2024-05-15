#include <SoftwareSerial.h>
//If the laser gives a reading above this distance, we recognize a pothole
const float maxLaserDistance = 1.11;
const int waitTime = 1000; //How long in milliseconds we wait before checking for a pothole again

const int rxPin1 = 2;
const int txPin1 = 3;
const int rxPin2 = 4;
const int txPin2 = 5;
SoftwareSerial mySerial_1 = SoftwareSerial(rxPin1, txPin1); // RX, TX
SoftwareSerial mySerial_2 = SoftwareSerial(rxPin2, txPin2);   // RX, TX

//The numbers were taken from online, not sure what they do exactly
char buff[4]={0x80,0x06,0x03,0x77}; //Hex for 128, 6, 3, 119
unsigned char data_laser_1[11]={0};
unsigned char data_laser_2[11]={0};

void setup() {
  pinMode(rxPin1, INPUT);
  pinMode(txPin1, OUTPUT);
  pinMode(rxPin2, INPUT);
  pinMode(txPin2, OUTPUT);
  Serial.begin(112500);        // Start the hardware serial port
  mySerial_1.begin(9600);     // Start software serial port 1
  mySerial_2.begin(9600);     // Start software serial port 2

}

void loop() {
  //These print statements are from the original code, but they don't seem to be necessary?
  //mySerial_1.print(buff);
  //mySerial_2.print(buff);

  checkLaser(mySerial_1, data_laser_1, "Left");
  checkLaser(mySerial_2, data_laser_2, "Right");
}

void checkLaser(SoftwareSerial myLaser, unsigned char laserData[], String laserSide) {

  myLaser.listen();
  delay(waitTime);
  if(myLaser.available()>0) 
  {

    delay(50);
    for(int i=0;i<11;i++){
      laserData[i] = myLaser.read();
    }
        
    unsigned char Check = 0;
        
    for(int i=0;i<10;i++){
      Check=Check+laserData[i];
    }
        
    Check=~Check+1;
    
    if(laserData[10]==Check){
      if(laserData[3]=='E'&& laserData[4]=='R' && laserData[5]=='R'){
        Serial.println("Out of range");
      }
      else{
      
        float distance0 = 0;
        distance0 = (laserData[3]-0x30)*100+(laserData[4]-0x30)*10+(laserData[5]-0x30)*1+(laserData[7]-0x30)*0.1+(laserData[8]-0x30)*0.01+(laserData[9]-0x30)*0.001;
        Serial.println("---------------");

        if(distance0 > maxLaserDistance) 
        {
          Serial.println("POTHOLE DETECTED (" + laserSide + ")");
        }
        else 
        {
          Serial.println("No pothole detected on " + laserSide);
        }
      }
    }
    else{
      Serial.println("Invalid Data on " + laserSide);
    }
  }
}
