#include <EEPROM.h>
// ----------------------- PIN DEFINITIONS -----------------------
// MOTOR pins for BTS7960 (PWM controlled)
#define left_motor_forward   16     // LPWM
#define left_motor_backward  17     // RPWM
#define right_motor_forward  2    // LPWM
#define right_motor_backward 4    // RPWM
// MUX pins for sensor selection
#define S0 25
#define S1 26
#define S2 32
#define S3 33
#define SIG_PIN 35
// Buttons
#define button1 5
#define button2 18
#define button3 23
#define button4 3
//others
#define buzzer 15
#define tr_sensor 19
#define led 27
#define servo1 12
#define servo2 14
// ----------------------- SENSOR VARIABLES -----------------------
#define sensorNumber 14
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192};
int WeightValue[sensorNumber] = {10,20,30,40,50,60,70,80,90,100,110,120,130,140};
int sumOnSensor;
int sensorWight;
int bitSensor;
int Max_ADC[sensorNumber];
int Min_ADC[sensorNumber];
int Reference_ADC[sensorNumber];
bool inverseON = 0;
// ----------------------- PID VARIABLES -----------------------
float line_position;
float error;
float previous_error;
float derivative;
int base_speed = 150;
int kp = 8;
int kd = 100;
float center_position = 75;
// ---------------------- LEDC CONFIG ---------------------------
#define LEDC_FREQ 20000
#define LEDC_RES  8
// LEDC channels
#define CH_LF 0
#define CH_LB 1
#define CH_RF 2
#define CH_RB 3
// ========================= SETUP ===============================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(tr_sensor, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(led, 0);
  digitalWrite(buzzer, 0);
  digitalWrite(tr_sensor, 1);
  // Configure LEDC channels
  ledcSetup(CH_LF, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_LB, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_RF, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_RB, LEDC_FREQ, LEDC_RES);
  // Attach PWM channels
  ledcAttachPin(left_motor_forward,  CH_LF);
  ledcAttachPin(left_motor_backward, CH_LB);
  ledcAttachPin(right_motor_forward, CH_RF);
  ledcAttachPin(right_motor_backward, CH_RB);
  LoadCalibration();
}

// ========================= MAIN LOOP ===========================
void loop() {
  if (!digitalRead(button1)) calibrateSensor();
  if (!digitalRead(button2)) PID_Controller(base_speed, kp, kd);
}

// ========================= MOTOR CONTROL =======================
void motor(int LPWM, int RPWM) {
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);
  // LEFT MOTOR
  if (LPWM > 0) {
    ledcWrite(CH_LF, LPWM);
    ledcWrite(CH_LB, 0);
  } else if (LPWM < 0) {
    ledcWrite(CH_LF, 0);
    ledcWrite(CH_LB, -LPWM);
  } else {
    ledcWrite(CH_LF, 0);
    ledcWrite(CH_LB, 0);
  }
  // RIGHT MOTOR
  if (RPWM > 0) {
    ledcWrite(CH_RF, RPWM);
    ledcWrite(CH_RB, 0);
  } else if (RPWM < 0) {
    ledcWrite(CH_RF, 0);
    ledcWrite(CH_RB, -RPWM);
  } else {
    ledcWrite(CH_RF, 0);
    ledcWrite(CH_RB, 0);
  }
}

// ========================= PID CONTROLLER ======================
void PID_Controller(int base_speed, int p, int d) {
  while (1) {
  a:
    read_black_line();
    if (sumOnSensor > 0) line_position = sensorWight / sumOnSensor;
    error = center_position - line_position;
    derivative = error - previous_error;
    int right_motor_value = base_speed + (error * p + derivative * d);
    int left_motor_value  = base_speed - (error * p + derivative * d);
    previous_error = error;
    motor(left_motor_value, right_motor_value);
    //inverse line detection
    if (sensorDigital[3]==1 && sensorDigital[10]==1 && bitSensor<16384) {
      inverseON = !inverseON;
      digitalWrite(led, inverseON);
      goto a;
    }
  }
}

// ========================= SENSOR CALIBRATION ==================
void calibrateSensor() {
  // Set initial Min–Max limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;
    Min_ADC[i] = 4095; // ESP32 ADC max
  }
  int rotationDirections[][2] = {
    {-1, 1},
    {1, -1},
    {1, -1},
    {-1, 1}
  };
  for (int step = 0; step < 4; step++) {
    motor(base_speed * 0.4 * rotationDirections[step][0],
          base_speed * 0.4 * rotationDirections[step][1]);

    for (int sweep = 0; sweep < 500; sweep++) {
      for (int i = 0; i < sensorNumber; i++) {
        selectChannel(i);
        delayMicroseconds(30);
        sensorADC[i] = analogRead(SIG_PIN);
        Max_ADC[i] = max(Max_ADC[i], sensorADC[i]);
        Min_ADC[i] = min(Min_ADC[i], sensorADC[i]);
      }
    }
    motor(0, 0);
    delay(200);
  }
  // Save into EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2;
    EEPROM.write(i, Reference_ADC[i] / 16);
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 16);
    EEPROM.write(i + sensorNumber * 2, Min_ADC[i] / 16);
  }
  EEPROM.commit();
}

// ========================= LOAD CALIBRATION ====================
void LoadCalibration() {
  Serial.println("Loading Calibration...");
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 16;
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 16;
    Min_ADC[i] = EEPROM.read(i + sensorNumber * 2) * 16;
    Serial.print(String(Reference_ADC[i]) + ", ");
  }
  Serial.println();
}

// ========================= READ SENSOR DATA ====================
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    // Black line detection
    if (sensorADC[i] > Reference_ADC[i])
      sensorDigital[i] = inverseON ? 0 : 1;
    else
      sensorDigital[i] = inverseON ? 1 : 0;
    sumOnSensor += sensorDigital[i];
    sensorWight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

// ========================= SELECT MUX CHANNEL ==================
void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

// ========================= DEBUG FUNCTIONS =====================
void Bit_Sensor_Show() {
  while (1) {
    read_black_line();

    Serial.print("Decimal: ");
    Serial.print(bitSensor);
    Serial.print(" | Binary: ");

    for (int i = sensorNumber - 1; i >= 0; i--)
      Serial.print(bitRead(bitSensor, i));

    Serial.println();
  }
}

