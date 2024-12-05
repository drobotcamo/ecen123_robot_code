#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define ENABLE_A 2 //Left
#define ENABLE_B 7//Right
#define IN_1 3
#define IN_2 4 
#define IN_3 8
#define IN_4 9
#define L_OUT_A 20
#define L_OUT_B 46
#define R_OUT_A 21
#define R_OUT_B 44
#define SWITCH_1 27
#define SWITCH_2 26
#define delay_options 1000

#define LED_BASE 22

#define LF_1 A2
#define LF_2 A3
#define LF_3 A4
#define LF_4 A5
#define LF_5 A6
#define LF_6 A7
#define LF_7 A8
#define LF_8 A9

#define SENSOR A15

#define RESTART_BUTTON 39

#define TOP_SPEED 255

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

const int DIST[8] = {5, 10, 15, 20, 25,30, 35, 40};
const float VOLT[8] = {3.42, 2.84, 1.92, 1.42, 1.11, 0.93, 0.74, 0.62};

const uint8_t LF_PINS[8] = {LF_1, LF_2, LF_3, LF_4, LF_5, LF_6, LF_7, LF_8};
QTRSensors qtr;

#define LF_CTRL 13

int b_value = 0;
int counter = 0;

bool flag = true; 

void setup(){
  pinMode(ENABLE_A, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(L_OUT_A, INPUT_PULLUP);
  pinMode(L_OUT_B, INPUT_PULLUP);
  pinMode(R_OUT_A, INPUT_PULLUP);
  pinMode(R_OUT_B, INPUT_PULLUP);
  pinMode(RESTART_BUTTON, INPUT_PULLUP);
  pinMode(SENSOR, INPUT);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  qtr.setTypeRC(); // or setTypeAnalog()
  qtr.setSensorPins(LF_PINS, 8);

  for(int i = 0; i < 8; i++) {
    pinMode(LED_BASE + i, OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(L_OUT_A), isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_OUT_A), isr2, RISING);
  
  Serial.begin(9600);

  digitalWrite(ENABLE_A, LOW);
  digitalWrite(ENABLE_B, LOW);

  calibrateLineFollower();
  linefollow();
}

void loop(){
  if (!digitalRead(RESTART_BUTTON)) {
    Serial.print("Received Restart Route command, restarting in 5 ");
    turnLEDS(HIGH);
    delay(1000);
    Serial.print("4 ");
    delay(1000);
    Serial.print("3 ");
    delay(1000);
    Serial.print("2 ");
    delay(1000);
    Serial.print("1 ");
    delay(1000);
    Serial.println("GO!");
    begin();
  }
}

#define CALIBRATION_STEPS 40

void calibrateLineFollower() {
  turnLEDS(LOW);
  Serial.println("Calibrating for White in 2 seconds");
  flashLEDs(2);

  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < CALIBRATION_STEPS; i++) {
    showProgressBar(i, CALIBRATION_STEPS);
    updateLEDS(i, CALIBRATION_STEPS);
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration for White Complete!");
  Serial.println();

  turnLEDS(LOW);
  delay(1000);

  Serial.println("Calibrating for Black in 2 seconds");
  flashLEDs(2);

  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < CALIBRATION_STEPS; i++) {
    showProgressBar(i, CALIBRATION_STEPS);
    updateLEDS(i, CALIBRATION_STEPS);
    qtr.calibrate();
    delay(20);
  }

  turnLEDS(LOW);
  Serial.println("Calibration for Black Complete!");
  Serial.println();
}

void begin() {
  unsigned long duration = 120000;
  unsigned long start = millis();
  uint16_t sensors[8];

  uint16_t position;
  int error;
  double previousError;

  int motorSpeed;
  int leftMotorSpeed;
  int rightMotorSpeed;

  const int defaultMotorSpeed = 155;
  // const double K_p = 0.03;
  const double K_p = 0.065;
  const double K_d = 0.06;

  while (millis() - start < duration) {
    position = qtr.readLineBlack(sensors);
    Serial.println(position);
    for(int i = 0; i < 8; i++) {
      if (sensors[i] > 800) {
        digitalWrite(LED_BASE + i, HIGH);
      } else {
        digitalWrite(LED_BASE + i, LOW);
      }
    }
    // Serial.println();
    error = 3500 - position;
    Serial.print("Error: ");
    Serial.println(error);

    motorSpeed = (int)(K_p * error + K_d * (error - previousError));
    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);

    previousError = error;

    motorSpeed = checkBounded(motorSpeed, -2 * defaultMotorSpeed, TOP_SPEED);

    leftMotorSpeed = defaultMotorSpeed - motorSpeed;
    rightMotorSpeed = defaultMotorSpeed + motorSpeed;

    leftMotorSpeed = checkBounded(leftMotorSpeed, -1 * defaultMotorSpeed, TOP_SPEED);
    rightMotorSpeed = checkBounded(rightMotorSpeed, -1 * defaultMotorSpeed, TOP_SPEED);
    Serial.print("Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right Motor Speed: ");
    Serial.println(rightMotorSpeed);

    if (rightMotorSpeed > 0) {
      r_motor(HIGH, LOW, HIGH, rightMotorSpeed);
    } else {
      r_motor(HIGH, HIGH, LOW, -1 * rightMotorSpeed);
    }

    if (leftMotorSpeed > 0) {
      l_motor(HIGH, HIGH, LOW, leftMotorSpeed);
    } else {
      l_motor(HIGH, LOW, HIGH, -1.0 * leftMotorSpeed);
    }
  }
  Brake();
}

void linefollow() {
  Serial.print("Starting in 5 ");
  delay(1000);
  Serial.print("4 ");
  delay(1000);
  Serial.print("3 ");
  delay(1000);
  Serial.print("2 ");
  delay(1000);
  Serial.print("1 ");
  delay(1000);
  Serial.println("GO!");
  begin();
}

int checkBounded(int input, int lowBound, int highBound) {
  if (input >= lowBound && input <= highBound) {
    return input;
  } else if (input < lowBound) {
    return lowBound;
  } else {
    return highBound;
  }
}

void route(){
  cmForward(20);
  delay(1000);
  cmForward(20);
  delay(1000);
  cmReverse(20);
}

void cmForward (int x){
  int start_count = counter;
  Forward();
  Serial.print("counts to complete distance: ");
  Serial.print(int(x / 0.06));
  
  int distance = 0;
  int currentTime = 0;

  int prevDist = 0;
  int prevMillis = 0;
  while (counter < start_count + int(x / 0.0194)) {
    Serial.print("Count:");
    Serial.println(counter);
    distance = counter * 0.0194;
    Serial.print("Distance:");
    Serial.println(distance);

    currentTime = millis();
    Serial.print("Speed:");
    Serial.println(((counter - prevDist)*0.0194 / (currentTime - prevMillis)) * 1000);
    prevDist = counter;
    prevMillis = currentTime;
  }

  Brake();
}

void cmReverse (int x){
  int start_count = counter;
  Reverse();
  Serial.print("Counts to complete distance: ");
  Serial.print(int(x / 0.06)); //

  int distance = 0;
  int currentTime = 0;

  int prevDist = 0;
  int prevMillis = 0;
  while (counter < start_count + int(x / 0.0194)) {
    Serial.print("Count:");
    Serial.println(counter);
    distance = counter * 0.06;
    Serial.print("Distance:");
    Serial.println(counter * 0.06);

    currentTime = millis();
    Serial.print("Speed:");
    Serial.println((counter * 0.06) / (currentTime - prevMillis));
    prevDist = distance;
    prevMillis = currentTime;

  }
  Brake();
}

void Forward(){
  motors(HIGH, HIGH, LOW, HIGH, LOW, HIGH);
}

void Reverse(){
  motors(HIGH, LOW, HIGH, HIGH, HIGH, LOW);
}

void Brake(){
  motors(HIGH, LOW, LOW, HIGH, LOW, LOW);
}

void Coast(){
  motors(LOW, LOW, LOW, LOW, LOW, LOW);
}

void TurnLeft(){
  motors(LOW, LOW, LOW, HIGH, LOW, HIGH);  //Left Motor - Coast, Right Motor - Forward
}

void TurnRight(){
  motors(HIGH, HIGH, LOW, LOW, LOW, LOW);  //Left Motor - Forward, Right Motor.- Coast
}

void PivotLeft(){
  motors(HIGH, LOW, LOW, HIGH, LOW, HIGH);  //Left Motor - Brake, Right Motor - Forward
}

void PivotRight(){
  motors(HIGH, HIGH, LOW, HIGH, LOW, LOW);  //Left Motor - Forward, Right Motor - Brake
}

void l_motor(int EN_A, int IN1, int IN2, int speed){
  digitalWrite(ENABLE_A, EN_A);
  analogWrite(IN_1, IN1 ? speed : 0);
  analogWrite(IN_2, IN2  ? speed : 0);
}

void r_motor(int EN_B, int IN3, int IN4, int speed){
  digitalWrite(ENABLE_B, EN_B);
  analogWrite(IN_3, IN3  ? speed : 0);
  analogWrite(IN_4, IN4  ? speed : 0);
}

void motors(int EN_A, int IN1, int IN2, int EN_B, int IN3, int IN4){
  int switchval1 = digitalRead(SWITCH_1);
  int switchval2 = digitalRead(SWITCH_2);
  
  int speed = 36;
  //Left Motor
  l_motor(EN_A,IN1, IN2, speed);

  //Right Motor
  r_motor(EN_B, IN3, IN4, speed); 
}

void isr()
{
  b_value = digitalRead(L_OUT_B);
  if (b_value) 
    counter++;
  else 
    counter--;
}

void isr2()
{
  b_value = digitalRead(R_OUT_B);
  if (!b_value) 
    counter++;
  else 
    counter--;
}

void showProgressBar(int step, int totalSteps) {
  int barWidth = 20; // Width of the progress bar in characters
  float progress = (float)step / totalSteps;
  int position = progress * barWidth;

  // Print the progress bar
  Serial.print("[");
  for (int i = 0; i < barWidth; i++) {
    if (i < position) {
      Serial.print("="); // Filled portion
    } else if (i == position) {
      Serial.print(">"); // Moving pointer
    } else {
      Serial.print(" "); // Empty space
    }
  }
  Serial.print("] ");
  Serial.print(int(progress * 100)); // Percentage completed
  Serial.println("%"); // Print percentage and move to the next line
}


//LED CODE
void turnLEDS(int state){
  for(int i = 0; i < 8; i++)
    digitalWrite(LED_BASE + i, state);
}

void updateLEDS(int step, int totalSteps){
  int totalLEDS = 8; 
  float progress = (float)step / totalSteps;
  int position = progress * totalLEDS;

  for (int i = 0; i < totalLEDS; i++){
    if (i < position)
      digitalWrite(LED_BASE + i, HIGH);
    else 
      digitalWrite(LED_BASE + i, LOW);    
  }
}


//RGB SENSOR CODE
int rgb_calc(){
  int time = millis();
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  //Determines White
  if(abs(r - b) < 250 && abs(r - g) < 250  && abs(b - g) < 250)
    return 0;

  //Determines Black
  else if(r < 1000 && g < 1000 && b < 1000)
    return 1;
  
  //Determines Red
  else if(r > g && r > b)
    return 2;

  //Determines Green
  else if(g > r && g > b)
    return 3;
  
  //Determine Blue
  else if(b > r && b > g)
    return 4;
  
  return -1;
}


//PROXIMITY SENSOR
int distance_calc(){
  int i, reading, distRight, distClose, dist; 
  float volt, voltRight, voltClose, slope;
  bool in_range = false;

  reading = analogRead(SENSOR);
  volt = reading * (5.0 / 1023.0);

  //The equation being used to calculate distance is X = ((x2 - x1) / (y2 - y1)) * (Y - y1) + x1
  for(i = 1; i < 7; i++)
    //We check to see if the voltage given by the sensor is between 
    if(volt > VOLT[i] && volt < VOLT[i - 1]){
      voltRight = VOLT[i];
      voltClose = VOLT[i - 1];
      distRight = DIST[i];
      distClose = DIST[i - 1];
      
      slope =  (distRight - distClose) / (voltRight - voltClose);
      dist = (slope * (volt - voltClose)) + distClose;

      in_range = true; 
      break;
    }
  
  //If the object is further than 40cm, it is considred out of range
  if(!in_range)
    return -1;

  return dist;
}

void flashLEDs(int count) {
  for(int i = 0; i < count; i++) {
    turnLEDS(HIGH);
    delay(500);
    turnLEDS(LOW);
    delay(500);
  }
}
