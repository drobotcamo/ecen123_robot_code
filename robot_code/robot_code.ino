#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// ============ BEGIN PIN DEFINITIONS ============
// Motor Pins
#define ENABLE_A 2 //Left
#define ENABLE_B 7 //Right
#define IN_1 3
#define IN_2 4
#define IN_3 8
#define IN_4 9
#define L_OUT_A 19
#define L_OUT_B 46
#define R_OUT_A 18
#define R_OUT_B 44
// End Motor Pins

// Line Follower Pins
#define LF_1 A2
#define LF_2 A3
#define LF_3 A4
#define LF_4 A5
#define LF_5 A6
#define LF_6 A7
#define LF_7 A8
#define LF_8 A9
const uint8_t LF_PINS[8] = {LF_1, LF_2, LF_3, LF_4, LF_5, LF_6, LF_7, LF_8};
// End Line Follower Pins

#define LED_BASE 22
#define G_LED 30
#define PROX_SENSOR A15
#define RESTART_BUTTON 39
// ============ END PIN DEFINITIONS ============


// ============ BEGIN DECLARATIONS ============
#define TOP_SPEED 255

// prox. sensor values
const int DIST[11] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55};
const float VOLT[11] = {3.11, 2.3, 1.53, 1.21, 1.03, 0.89, 0.73, 0.69, 0.66, 0.61, 0.57};

// color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// line follower
QTRSensors qtr;
#define LF_CALIB_STEPS 40

int b_value = 0;
int counter = 0;

enum MOLE_COLORS {
  GREEN,
  BLUE,
  WHITE,
  RED,
  PURPLE,
  YELLOW
};
// ============ END DECLARATIONS ============

void setup(){
  Serial.begin(9600);

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
  pinMode(PROX_SENSOR, INPUT);
  for(int i = 0; i < 8; i++) {
    pinMode(LED_BASE + i, OUTPUT);
  }
  pinMode(G_LED, OUTPUT);

  // initialize color sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // initialize line follower
  qtr.setTypeRC(); // or setTypeAnalog()
  qtr.setSensorPins(LF_PINS, 8);

  // initialize motor encoder interrupts (distance tracking)
  attachInterrupt(digitalPinToInterrupt(L_OUT_A), isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_OUT_A), isr2, RISING);

  // make sure motors are off
  digitalWrite(ENABLE_A, LOW);
  digitalWrite(ENABLE_B, LOW);

  // calibrateLineFollower();
  // linefollow();

  Serial.println("Starting in 2 seconds");
  delay(2000);
  //calibrate_RGB();
}

void loop(){
  //if (!digitalRead(RESTART_BUTTON)) {
    // loop code here maybe?
  //}
  calibrateLineFollower();
}

// ============ BEGIN LINE FOLLOWER CODE ============
// this method will attempt to follow a line for ~duration~ milliseconds
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

// calibrate the line follower in two phases -> first expose all of
// the LF sensors to the darkest color they will see (black tape) and
// then to the lightest color (wood)
// note: it's important that every one of the eight sensors sees
// black, not just the middle few.
void calibrateLineFollower() {
  turnLEDS(LOW);
  // calibrate for black
  Serial.println("Calibrating for Black");
  digitalWrite(G_LED, HIGH);
  flashLEDs(3);
  digitalWrite(G_LED, LOW);
  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < LF_CALIB_STEPS; i++) {
    showProgressBar(i, LF_CALIB_STEPS);
    updateLEDS(i, LF_CALIB_STEPS);
    qtr.calibrate();
    delay(10);
  }
  Serial.println("Calibration for Black Complete!");
  Serial.println();
  turnLEDS(LOW);
  delay(300);

  cmReverse(3);
  pivotDegrees(10);

  Serial.println("Calibrating for White");
  flashLEDs(3);
  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < LF_CALIB_STEPS; i++) {
    showProgressBar(i, LF_CALIB_STEPS);
    updateLEDS(i, LF_CALIB_STEPS);
    qtr.calibrate();
    delay(20);
  }
  turnLEDS(LOW);

  Serial.println("Calibration for White Complete!");
  Serial.println();
}

// starts line following 5 seconds from when called
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

// simple route
void route(){
  cmForward(20);
  delay(1000);
  cmReverse(20);
  delay(1000);
}
// ============ END LINE FOLLOWER CODE ============

// ============ BEGIN MOTOR CODE ============
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
  while (counter > start_count - int(x / 0.0194)) {
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

  int speed = 36;
  //Left Motor
  l_motor(EN_A,IN1, IN2, speed);

  //Right Motor
  r_motor(EN_B, IN3, IN4, speed);
}

// interrupt that will count movements on the left motor
void isr()
{
  b_value = digitalRead(L_OUT_B);
  if (b_value)
    counter--;
  else
    counter++;
}

// interrupt that will count movements on the right motor
void isr2()
{
  b_value = digitalRead(R_OUT_B);
  if (!b_value)
    counter--;
  else
    counter++;
}
// ============ END MOTOR CODE ============


// ============ BEGIN LED CODE ============
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

void flashLEDs(int count) {
  for(int i = 0; i < count; i++) {
    turnLEDS(HIGH);
    delay(100);
    turnLEDS(LOW);
    delay(100);
  }
}
// ============ END LED CODE ============


// ============ BEGIN RGB SENSOR CODE ============
// returns the name of a color as a String
String getColorName(MOLE_COLORS color) {
  switch (color) {
    case GREEN: return "Green";
    case BLUE: return "Blue";
    case WHITE: return "White";
    case RED: return "Red";
    case PURPLE: return "Purple";
    case YELLOW: return "Yellow";
    default: return "Unknown";
  }
}

#define NUM_RGB_POINTS 10
// Tester for recording RGB data to the Serial Monitor. This method will ask the user
// what color is currently on the moles. Type the color and hit enter, then it will
// record NUM_RGB_POINTS data points for that color and print them.
void calibrate_RGB() {
  int index = 0;

  MOLE_COLORS currentColor;

  while (true) {
    Serial.println("Which color is being recorded right now?");
    Serial.println("Options: GREEN, BLUE, WHITE, RED, PURPLE, YELLOW");
    Serial.println("Type the color name (case-insensitive) or 'quit' to exit.");

    while (true) {
      if (Serial.available() > 0) {
        String message = Serial.readStringUntil('\n');
        message.trim();
        message.toLowerCase();

        if (message.equals("quit")) {
          Serial.println("Quitting calibration.");
          return;
        } else if (message.equals("green")) {
          currentColor = GREEN;
          break;
        } else if (message.equals("blue")) {
          currentColor = BLUE;
          break;
        } else if (message.equals("white")) {
          currentColor = WHITE;
          break;
        } else if (message.equals("red")) {
          currentColor = RED;
          break;
        } else if (message.equals("purple")) {
          currentColor = PURPLE;
          break;
        } else if (message.equals("yellow")) {
          currentColor = YELLOW;
          break;
        } else {
          Serial.println("Invalid color. Please type one of: GREEN, BLUE, WHITE, RED, PURPLE, YELLOW.");
        }
      }
    }
    for (int i = 0; i < NUM_RGB_POINTS; i++) {
      get_RGB_data_point(&index, currentColor);
      delay(100); // Collect data with a small delay
    }
  }
}

// get and record one RGB data point
void get_RGB_data_point(int* index, MOLE_COLORS color) {
  int time = millis();
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  record_RGB_data(*index, r, g, b, c, colorTemp, lux, time, color);

  (*index) += 1;
}

// method that will print out the RGB values to the serial monitor in CSV row format
void record_RGB_data(int index, int r, int g, int b, int c, int colorTemp, int lux, int time, MOLE_COLORS color) {
  Serial.print(index); Serial.print(",");
  Serial.print(r); Serial.print(",");
  Serial.print(g); Serial.print(",");
  Serial.print(b); Serial.print(",");
  Serial.print(c); Serial.print(",");
  Serial.print(colorTemp); Serial.print(",");
  Serial.print(lux); Serial.print(",");
  Serial.print(getColorName(color)+",");
  Serial.print(time);
  Serial.println();
}

// algorithm that will decide which color the sensor is looking at
MOLE_COLORS get_current_observed_color(){
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  if (b < 5000) {
        if (r < 3000) {
            return GREEN;
        } else {
            if (lux < 638) {
                return RED;
            } else {
                return YELLOW;
            }
        }
  } else {
        if (r < 2000) {
            return BLUE;
        } else {
            if (lux < 0) {
                return PURPLE;
            } else {
                return WHITE;
            }
        }
  }
}
// ============ END RGB SENSOR CODE ============


// ============ BEGIN PROXIMITY SENSOR CODE ============
int distance_calc(){
  int i, reading, distRight, distClose, dist;
  float volt, voltRight, voltClose, slope;
  bool in_range = false;

  reading = analogRead(PROX_SENSOR);
  volt = reading * (5.0 / 1023.0);

  //The equation being used to calculate distance is X = ((x2 - x1) / (y2 - y1)) * (Y - y1) + x1
  for(i = 1; i < 11; i++)
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
// ============ END PROXIMITY SENSOR CODE ============


// ============ BEGIN MISC. CODE ============
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

int checkBounded(int input, int lowBound, int highBound) {
  if (input >= lowBound && input <= highBound) {
    return input;
  } else if (input < lowBound) {
    return lowBound;
  } else {
    return highBound;
  }
}
// ============ END MISC. CODE ============
