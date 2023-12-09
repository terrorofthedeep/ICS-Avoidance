#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3
SoftwareSerial mySerial = SoftwareSerial(rxPin,txPin);//Define software serial, 3 is TX, 2 is RX
char buff[4]={0x80,0x06,0x03,0x77};
unsigned char data[11]={0};
void setup()
{
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
 Serial.begin(115200);
 mySerial.begin(9600);
}

void loop()
{
  mySerial.print(buff); // Error Candidate 2
  while(1)
  {
    if(mySerial.available()>0)//Determine whether there is data to read on the serial 
    {
      
      Serial.println("data ready");
      delay(100);
      for(int i=0;i<11;i++)
      {
        data[i]=mySerial.read(); // Error Candidate 1
      }
      unsigned char Check=0;
      for(int i=0;i<10;i++)
      {
        Check=Check+data[i];
      }
      Check=~Check+1;
      if(data[10]==Check)
      {
        if(data[3]=='E'&&data[4]=='R'&&data[5]=='R')
        {
          Serial.println("Out of range");
        }
        else
        {
          float distance=0;
          distance=(data[3]-0x30)*100+(data[4]-0x30)*10+(data[5]-0x30)*1+(data[7]-0x30)*0.1+(data[8]-0x30)*0.01+(data[9]-0x30)*0.001;
          Serial.print("Distance = ");
          Serial.print(distance,3);
          Serial.println(" M");
          if(distance > 1.11) 
          {
            Serial.print("Pothole detected");
          }
          else 
          {
            Serial.print("No pothole detected");
          }
      }
      else
      {
        Serial.println("Invalid Data!");
      }
    }else{
      Serial.println("data error!");
    }
    delay(40);
  }
}
