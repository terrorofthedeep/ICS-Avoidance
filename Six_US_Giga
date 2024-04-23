#include "NewPing.h"

//Acceptable distances
const int frontMaxDist = 400;
const int sideMaxDist = 30;

// defines pins numbers
const int trigPin_1 = 23;
const int trigPin_2 = 25;
const int trigPin_3 = 27;
const int trigPin_4 = 29;
const int trigPin_5 = 31;
const int trigPin_6 = 33;

const int echoPin_g = 22;
const int echoPin_w = 24;
const int echoPin_y = 26;
const int echoPin_o = 28;
const int echoPin_b = 30;
const int echoPin_p = 32;


NewPing sonar_1(trigPin_1, echoPin_g, frontMaxDist);
NewPing sonar_2(trigPin_2, echoPin_w, frontMaxDist);
NewPing sonar_3(trigPin_3, echoPin_y, frontMaxDist);
NewPing sonar_4(trigPin_4, echoPin_o, frontMaxDist);
NewPing sonar_5(trigPin_5, echoPin_b, frontMaxDist);
NewPing sonar_6(trigPin_6, echoPin_p, frontMaxDist);

void setup() {
  
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  Serial.print("Distance_1 = ");
  Serial.print(sonar_1.ping_cm()); // How to get distance
  Serial.println(" cm");
  
  Serial.print("Distance_2 = ");
  Serial.print(sonar_2.ping_cm());
  Serial.println(" cm");
  
  Serial.print("Distance_3 = ");
  Serial.print(sonar_3.ping_cm());
  Serial.println(" cm");
  
  Serial.print("Distance_4 = ");
  Serial.print(sonar_4.ping_cm());
  Serial.println(" cm");
  
  Serial.print("Distance_5 = ");
  Serial.print(sonar_5.ping_cm());
  Serial.println(" cm");
  
  Serial.print("Distance_6 = ");
  Serial.print(sonar_6.ping_cm());
  Serial.println(" cm");
  //delay(50);
}
