//MULTIPLEXER CONTROL PINS (ESP32)
#define S0 16
#define S1 17
#define S2 5
#define S3 18
#define SIG_PIN 4
#define sensorNumber 14
int sensor[sensorNumber];

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);
  // Serial communication
  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(5);  // small settling delay
    sensor[i] = analogRead(SIG_PIN);
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.println();
}

//SELECT MUX CHANNEL
void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));  // LSB
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));  // MSB
}
