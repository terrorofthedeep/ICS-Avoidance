#include <SoftwareSerial.h>
//If the laser gives a reading above this distance, we recognize a pothole
const float maxLaserDistance = 1.11;

#define rxPin1 2
#define txPin1 3
#define rxPin2 4
#define txPin2 5
SoftwareSerial mySerial_1 = SoftwareSerial(rxPin1, txPin1); // RX, TX
SoftwareSerial mySerial_2 = SoftwareSerial(rxPin2, txPin2);   // RX, TX

char buff[4]={0x80,0x06,0x03,0x77};
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

  mySerial_1.print(buff);
  mySerial_2.print(buff);
  while(1){ 
    mySerial_1.listen();
    delay(50);
    if(mySerial_1.available()>0) 
    {
      
      delay(50);
      for(int i=0;i<11;i++){
        data_laser_1[i]=mySerial_1.read();
      }
      
      unsigned char Check=0;
      
      for(int i=0;i<10;i++){
        Check=Check+data_laser_1[i];
      }
      
      Check=~Check+1;
      
      if(data_laser_1[10]==Check){
        if(data_laser_1[3]=='E'&& data_laser_1[4]=='R' && data_laser_1[5]=='R'){
          Serial.println("Out of range");
        }
        else{
        
          float distance0 = 0;
          distance0=(data_laser_1[3]-0x30)*100+(data_laser_1[4]-0x30)*10+(data_laser_1[5]-0x30)*1+(data_laser_1[7]-0x30)*0.1+(data_laser_1[8]-0x30)*0.01+(data_laser_1[9]-0x30)*0.001;
          Serial.print("Distance = ");
          Serial.print(distance0,3);
          Serial.println(" M");

          if(distance0 > maxLaserDistance) 
          {
            Serial.print("Pothole detected");
          }
          else 
          {
            Serial.print("No pothole detected");
          }
        }
      }
      else{
        Serial.println("Invalid Data!");
      }
    } 
    
    
    mySerial_2.listen();
    delay(50);
    if(mySerial_2.available()>0) 
    {
      
      delay(50);
      for(int i=0;i<11;i++){
        data_laser_2[i]=mySerial_2.read();
      }
      
      unsigned char Check=0;
      
      for(int i=0;i<10;i++){
        Check=Check+data_laser_2[i];
      }
      
      Check=~Check+1;
      
      if(data_laser_2[10]==Check){
        if(data_laser_2[3]=='E'&& data_laser_2[4]=='R' && data_laser_2[5]=='R'){
          Serial.println("Out of range");
        }
        else{
        
          float distance1 = 0;
          distance1=(data_laser_2[3]-0x30)*100+(data_laser_2[4]-0x30)*10+(data_laser_2[5]-0x30)*1+(data_laser_2[7]-0x30)*0.1+(data_laser_2[8]-0x30)*0.01+(data_laser_2[9]-0x30)*0.001;
          Serial.print("Distance2 = ");
          Serial.print(distance1,3);
          Serial.println(" M");
          
          if(distance1 > maxLaserDistance) 
          {
            Serial.print("Pothole detected");
          }
          else 
          {
            Serial.print("No pothole detected");
          }

          
        }
      }
      else{
        Serial.println("Invalid Data for mySerial_2");
      }
    }
  }
}
