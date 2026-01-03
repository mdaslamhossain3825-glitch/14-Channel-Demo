// Define control pins for the multiplexer
#define S0 8        //13//11
#define S1 10       //12//10
#define S2 11       //14//8
#define S3 12       //27//7
#define SIG_PIN A7  //26//A7  // Define the signal pin connected to the multiplexer (SIG)
#define sensorNumber 14
int sensor[sensorNumber];

void setup() {
  // Set control pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);
  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    sensor[i] = analogRead(SIG_PIN);
    Serial.print(String(sensor[i]) + "  ");
  }
  Serial.println();
}

// Function to select a channel on the multiplexer
void selectChannel(int channel) {
  // Set the state of each control pin
  digitalWrite(S0, bitRead(channel, 0));  // Least significant bit
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));  // Most significant bit
}
