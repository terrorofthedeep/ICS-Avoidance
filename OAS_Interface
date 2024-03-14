#include <SPI.h>

// Define pins for communication
const int IN_DATA_PIN = 2;
const int IN_VALID_PIN = 3;
const int IN_RDY_PIN = 4;
const int OUT_DATA_PIN = 5;
const int OUT_VALID_PIN = 6;
const int OUT_RDY_PIN = 7;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set pin modes
  pinMode(IN_DATA_PIN, INPUT);
  pinMode(IN_VALID_PIN, INPUT);
  pinMode(OUT_DATA_PIN, OUTPUT);
  pinMode(OUT_VALID_PIN, OUTPUT);
  pinMode(IN_RDY_PIN, INPUT);
  pinMode(OUT_RDY_PIN, OUTPUT);
}

void loop() {
  // Read input signals from FPGA
  int inData = digitalRead(IN_DATA_PIN);
  int inValid = digitalRead(IN_VALID_PIN);

  // Process input data if valid
  if (inValid == HIGH) {
    // Process inData
    // For example, send data to serial monitor
    Serial.println("Received data from FPGA: " + String(inData));
  }

  // Check if FPGA is ready to receive new data
  int inRdy = digitalRead(IN_RDY_PIN);
  if (inRdy == HIGH) {
    // Send data to FPGA
    digitalWrite(OUT_DATA_PIN, 1);  // Assuming 1 is the data to be sent
    digitalWrite(OUT_VALID_PIN, HIGH);  // Signal that data is valid
  } else {
    digitalWrite(OUT_VALID_PIN, LOW);  // Signal that data is not valid
  }

  // Check if Arduino is ready to receive new data
  if (client.available() > 0) {
    int outData = client.read();  // Read data from WiFi client
    digitalWrite(OUT_RDY_PIN, HIGH);  // Signal that Arduino is ready to receive
    Serial.println("Sending data to FPGA: " + String(outData));
  } else {
    digitalWrite(OUT_RDY_PIN, LOW);  // Signal that Arduino is not ready to receive
  }

  // Handle other tasks here
}
