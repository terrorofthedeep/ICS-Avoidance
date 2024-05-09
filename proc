#include <WiFi.h>
#include <Servo.h>
#include "mbed.h"
#include "oas.h"

// Wifi variables
char ssid[] = "TP-Link_2F9C";
char pass[] = "1cs_Pr0c";
IPAddress ip(192, 168, 0, 22);
String vehId = "veh2";
int status = WL_IDLE_STATUS;
WiFiServer breadcrumbListener(80); // Web server that listens on port 80
char followerVehicle[] = "Default IP"; // Web server ip that the leader vehicle will connect to once given by controller
int followerID; // vehicle ID number
String readString;
WiFiClient client;

// Flag variables
bool isLeader;
bool gotFollower;
bool connected = false;
bool second_loop = false;

// constants for Motor control
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

char leader_ip[] = "";
char follower_ip[] = "";

////////////////////////
///Motor control code///
////////////////////////

// takes values between 60 and 120
// Needs different frequency that analog write does not provide (330 Hz)
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

// takes values between 0-255 (0 is stopped)
void vehicleSpeed(int speed) {
  // Serial.println(speed);
  analogWrite(pinSpeed,speed);
}

///////////
///Setup///
///////////
void setup() {
  //gotFollower = false;
  //connected = false;
  //second_loop = false;
  Serial.begin(9600);
  delay(5000);
  Serial.println("STARTING");
  delay(5000);

  avoid_setup();
  /*
  if (WiFi.status() == WL_NO_MODULE) {
    // Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  
  wifiInit();

  // followerVehicle = ;
  while(!gotFollower) {
    // Serial.println("Getting follower vehicle");
    receive(0);
  }
  */
}
/*
void wifiInit() {
  if (WiFi.status() == WL_NO_MODULE) {
    // Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

// Set Static IP
  WiFi.config(ip);
// attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    // Serial.print("Attempting to connect to SSID: ");
    // Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 5 seconds for connection:
    delay(5000);
  }

  breadcrumbListener.begin();
  // Serial.print("SSID: ");
  // Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  // Serial.print("IP Address: ");
  // Serial.println(ip);
}

void receive(bool getBreadcrumb) {
  // status = WiFi.status();
  // while (status != WL_CONNECTED) {
  //     // Serial.print("Attempting to connect to SSID: ");
  //     // Serial.println(ssid);
  //     // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  //     status = WiFi.begin(ssid, pass);

  //     // wait 10 seconds for connection:
  //     // delay(2000);
  //   }
  while(!connected){
    WiFiClient client = breadcrumbListener.available();
    if (client) {
      connected = true;
      // Serial.println("new client");

      while (client.connected())
      {
        if (client.available())
        {
          char c = client.read();
          if (readString.length() < 100)
          {
            // If connection is connected and available, start storing the message
            readString += c;
            Serial.write(c);

            if (c == '\n') {
              if (readString.indexOf("/?kill") > 0) {
                // In the case of a soft kill
                vehicleAngle(90);
                vehicleSpeed(0);
                //if (client.connected()) {
                //  client.stop();
                //}
                delay(1);
                setup();
              } else if(readString.indexOf(vehId) > 0) {
                // Ensure the message is being sent to the right vehicle
                if(getBreadcrumb) {
                  parseBreadcrumb();
                }
                else {
                  parseVehSel();
                }
              }
            
              readString = ""; // reset the temporary read string

              delay(1);
            // client.stop();
            // Serial.println("client disconnected");
            }
          }
        }
      }
    }
    // Serial.println("not recieved");
  // delay(500);
  }
  second_loop = false;
  connected = false;
}

void parseVehSel() {
  String ipPrefix = String("192.168.0.");
  String followerIDString = String("");
  // if (readString.indexOf(vehId) > 0) {
    gotFollower = true;
    int leaderIndex = readString.indexOf("leader: ") + 8; // Locate the start of the leader flag value
    int followerIndex = readString.indexOf("follower: ") + 10; // Locate the start of the follower value

    // Extract the leader flag boolean
    String leaderString = readString.substring(leaderIndex, readString.indexOf(" ", leaderIndex));
    isLeader = leaderString.toInt(); // Convert leader string to flag value 

    // Extract the angle value
    String followerString = readString.substring(followerIndex, readString.indexOf(" ", followerIndex));
    int followerOffset = followerString.toInt();
    followerID = followerOffset + 20; // Convert follower string to get its ip lsB
    followerIDString = String(followerID); // convert lsB value to string
    String followerVehicleString = String(ipPrefix + followerIDString); // concat the lsB to the prefix 
    followerVehicleString.toCharArray(followerVehicle, followerVehicleString.length() + 1); // Convert to char array so it can be sent to connect() function

    // Output the extracted values
    // Serial.print("Is a leader: ");
    // Serial.println(isLeader);
    // Serial.print("Follower IP: ");
    // Serial.println(followerVehicle);
    delay(1);
  // }
}

void parseBreadcrumb() {
  // if (readString.indexOf(vehId) > 0) {
    int speedIndex = readString.indexOf("speed:") + 7; // Locate the start of the speed value
    int angleIndex = readString.indexOf("angle:") + 7; // Locate the start of the angle value

    // Extract the speed value
    String speedString = readString.substring(speedIndex, readString.indexOf(" ", speedIndex));
    speed = speedString.toInt(); // Convert speed string to an integer

    // Extract the angle value
    String angleString = readString.substring(angleIndex);
    angle = angleString.toInt(); // Convert angle string to an integer

    // Output the extracted values
    // Serial.print("Speed: ");
    // Serial.println(speed);
    // Serial.print("Angle: ");
    // Serial.println(angle);

    if(!isLeader) {
      avoid(angle, speed);
    }
    vehicleAngle(angle);
    vehicleSpeed(speed);
    delay(1);
  // }
}

void send(int speed, int angle) {
  String message = "";
  String speedStr = "";
  String angleStr = "";
  String followerVehString = "";
  while(!second_loop){
    // Serial.println(followerVehicle);
    if (client.connect(followerVehicle, 80)) {
      second_loop = true;
      // Serial.println("connected");
      speedStr = String(speed);
      angleStr = String(angle);
      followerVehString = String(followerID - 20);

      message = String("POST /?veh" + followerVehString + " speed: " + speedStr + " angle: " + angleStr + " HTTP/1.0");
      client.println(message);
      client.println();
    }else{
      // Serial.println("Not connected");
    }

    //if (client.connected()) {
    //  client.stop();
    //}
    // delay(5000);
  }
  second_loop = false;
  // Serial.println("send loop left");
  // delay(5000);
}
*/
//////////
///LOOP///
//////////
void loop() {
  /*
  receive(1);
  if(followerID != 37) {
    send(speed, angle);
  }
  */
  speed = 50;
  angle = 90;
  int *speedPtr = &speed;
  int *anglePtr = &angle;

  avoid(anglePtr, speedPtr);
  vehicleAngle(*anglePtr);
  vehicleSpeed(*speedPtr);
  
  Serial.println("ICS_OAS angle: "+ String(*anglePtr));
  Serial.println("-------------------------------------------------------------");
  
  //delay(5000);

}
