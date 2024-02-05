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
const int echoPin_h = 6; //Ready signal pin
// defines variables
long duration;
int distance; 
int breadCrumb[3] = {160, 20, 10};

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin_h, OUTPUT); // Sets pin 6 as output to be used a Ready Signal
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
  digitalWrite(echoPin_h, LOW); //Set Ready signal to LOW

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
    
    // Display Data
    Serial.print(" Sensor ");
    Serial.print(i - 6);
    Serial.print(": ");
    Serial.print(distance);
    Serial.print(", ");
    sensors[i - 7] = distance;    
  }
  Serial.println("");
  Serial.print("BreadCrumb");
  for (int i = 0; i < 3; i++){
    Serial.print(breadCrumb[i]);
    Serial.print(", ");
  }
  Serial.println(" ");
 
  int adjBreadCrumb[3];
  avoid_2(sensors,breadCrumb,adjBreadCrumb);
  Serial.print("AdjBreadCrumb");
  for (int i = 0; i < 3; i++){
    Serial.print(adjBreadCrumb[i]);
    Serial.print(", ");
  }
  Serial.println("");
  digitalWrite(echoPin_h, HIGH) //set Ready signal to HIGH (We're done processing breadcrumb)

  delay(1000);
}

// ----------NOTE----------
// The sensor will read zero if we did not get a good reading
// IF that is the case then we need to ignore
// ------------------------

// Purpose: To determine if the vehicle is off track and to readjust the heading
// Input  : distances (an array of sensor distances)
// Output : NULL
void avoid(int distances[]) {
  //Check if we can continue straight
  if(distances[2] > frontMaxDist && distances[3] > frontMaxDist) {
    Serial.print("Go forward");
  }

  //Basic outline:
  //If sensors 1-3 are clear, slightly left
  //If sensors 1-2 are clear, significantly left
  //If sensor 1 is clear, hard left
  //If sensors 4-6 are clear, slightly right
  //If sensors 5-6 are clear, significantly right
  //If sensor 6 is clear, hard right

  
  //If the left side is all clear
  else if(distances[0] > sideMaxDist && distances[1] > sideMaxDist && distances[2] > frontMaxDist) {
        Serial.print("Go slightly left");
  }
  //If the leftmost two are clear
  else if(distances[0] > sideMaxDist && distances[1] > sideMaxDist) {
        Serial.print("Go significantly left");
  }
  else if(distances[0] > sideMaxDist) {
            Serial.print("Go hard left");
  }
  //If the right side is all clear
  else if(distances[5] > sideMaxDist && distances[4] > sideMaxDist && distances[3] > frontMaxDist) {
        Serial.print("Go slightly right");
  }
  //If the rightmost two are clear
  else if(distances[5] > sideMaxDist && distances[4] > sideMaxDist) {
        Serial.print("Go significantly right");
  }
  else if(distances[5] > sideMaxDist) {
            Serial.print("Go hard right");
  }
  //If there are objects in all directions, stop
  else {
    Serial.print("Stop");
  }
}

void avoid_2(int Sensor[], int breadcrumb[3], int result[3]) {
    // Grab the breadcrumb
    int Angle = breadcrumb[0];
    int Distance = breadcrumb[1];
    int Speed = breadcrumb[2];

    // Determine which angle for the breadcrumb
    int Direction[] = {0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150};
    int SensorIdx[] = {0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5};

    // Determine which sensor we need to start with
    int startSens = (Angle) / 30;
    Serial.print("StartSens");
    Serial.println(startSens + 1);
    for (int i = 0; i < 11; i++) {
        if (Angle % 30 == 0 && i == 0 && Angle < 150 && Angle > -150) {
            if (Sensor[startSens] > Distance && Sensor[startSens - 1] > Distance) {
                result[0] = Angle;
                result[1] = Distance;
                result[2] = Speed;
                return;
            }
        }
        else if ((startSens + SensorIdx[i] >= 0) && (startSens + SensorIdx[i] < 6)) {
            if (Sensor[startSens + SensorIdx[i]] > Distance) {
                result[0] = Angle + Direction[i];
                result[1] = Distance;
                result[2] = Speed;
                return;
            }
        }
    }
    result[0] = -1;
    result[1] = -1;
    result[2] = -1;
}
