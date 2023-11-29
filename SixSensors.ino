//Acceptable distances
const int frontMaxDist = 200;
const int sideMaxDist = 30;

// defines pins numbers
const int trigPin = 13;
const int echoPin_p = 12;
const int echoPin_b = 11;
const int echoPin_o = 10;
const int echoPin_y = 9;
const int echoPin_w = 8;
const int echoPin_g = 7;
// defines variables
long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_p, INPUT); // Sets the echoPin as an Input
  pinMode(echoPin_b, INPUT);
  pinMode(echoPin_o, INPUT);
  pinMode(echoPin_y, INPUT);
  pinMode(echoPin_w, INPUT);
  pinMode(echoPin_g, INPUT);
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  Serial.println("New Iteration");
  int sensors[6];
  for(int i = 7; i < 13; i++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); 
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(i, HIGH);
    distance = duration * 0.034 / 2;
    Serial.print(" Sensor ");
    Serial.print(i - 6);
    Serial.print(": ");
    Serial.print(distance);
    Serial.print(", ");
    sensors[i - 7] = distance;
  }
  Serial.println("");
  avoid(sensors);

  Serial.println("");
  
  
  delay(1000);
}

void avoid(int distances[]) {
  //Check if we can continue straight
  if(distances[2] > frontMaxDist && distances[3] > frontMaxDist) {
    
    Serial.print("Go straight");
  }
  //Check if we can turn left
  else if (distances[1] > sideMaxDist) {
    if (distance[0] < sideMaxDist){
      Serial.print("Go 45 degrees left);
    } else{
      Serial.print("Go 90 degrees left");
    }
  }
  //Check if we can turn right
  else if(distances[4] > sideMaxDist && distances[5] > sideMaxDist) {
    if (distance[5] < sideMaxDist){
      Serial.print("Go 45 degrees right);
    } else{
      Serial.print("Go 90 degrees right");
    }
  }
  //If there are objects in all directions, stop
  else {
    Serial.print("Stop");
  }
}
