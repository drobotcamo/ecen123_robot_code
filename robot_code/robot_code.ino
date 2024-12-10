#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>

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
#define ARM_SERVO 10
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
//#define G_LED 30
//#define R_LED 31
#define PROX_SENSOR A15
#define RESTART_BUTTON 39

// LED pins for color sensor
#define Y_LED_C 36
#define P_LED_C 35
#define R_LED_C 34
#define W_LED_C 32
#define B_LED_C 31
#define G_LED_C 30

// ============ END PIN DEFINITIONS ============


// ============ BEGIN DECLARATIONS ============
#define TOP_SPEED 255

// prox. sensor values
const int DIST[11] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55};
const float VOLT[11] = {3.11, 2.3, 1.53, 1.21, 1.03, 0.89, 0.73, 0.69, 0.66, 0.61, 0.57};

// color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
#define NUM_COLOR_SAMPLES 3

const int MOLE_DEGREES[7] = {-90, -60, -30, 0, 30, 60, 90};

// line follower
QTRSensors qtr;
#define LF_CALIB_STEPS 40

// servo arm
Servo myservo;
#define SERVO_STARTING_POINT 0
#define SERVO_ENDING_POINT 100

int b_value = 0;
int counter = 0;

enum MOLE_COLORS {
  GREEN,
  BLUE,
  WHITE,
  BUTTON,
  RED,
  PURPLE,
  YELLOW
};
int LED_base = 30;
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
  for(int i = 0; i < 7; i++)
    pinMode(LED_base + i, OUTPUT);

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

  // declare servo
  myservo.attach(ARM_SERVO);
  myservo.write(SERVO_STARTING_POINT);

  // make sure motors are off
  digitalWrite(ENABLE_A, LOW);
  digitalWrite(ENABLE_B, LOW);

  // safety delay
  Serial.println("Starting in 2 seconds");
  customDelay(2000);

  master_route();
}

void loop(){
  //if (!digitalRead(RESTART_BUTTON)) {
    // loop code here maybe?
  //}
  // calibrateLineFollower();
}

// ============ BEGIN CONTROL CODE ============
void master_route(){
  // calibrate
  calibrateLineFollower();
  customDelay(1000);
  // follow line until coins
  linefollow_duration(11000);

  turn_and_hit_coins();
  customDelay(250);
  linefollow_duration(1350);
  turn_and_hit_coins();

  millisecondsReverse(1000);
  // face button
  pivotDegrees(-90,255);
  customDelay(1000);
  // drive to line
  millisecondsForward(1500);
  customDelay(1000);
  // hit button
  linefollow_duration(2500);
  linefollow_until_wall_dist(10);
  millisecondsForward(1000);
  customDelay(1000);
  // go back
  millisecondsReverse(3000);
  customDelay(1000);

  MOLE_COLORS currentColor = BUTTON;
  MOLE_COLORS nextColor = RED;

  unsigned long start = millis();
  unsigned long duration = 120000;
  while(millis() - start < duration){
     whacAMole(currentColor, nextColor);
     millisecondsForward(1833);
    //  linefollow_duration(1300);
    linefollow_until_wall_dist(7);
     whack_mole();
     customDelay(100);
     digitalWrite(LED_base + currentColor, LOW);
     currentColor = nextColor;
     nextColor = see_the_color();
     // check for color read error (purple vs blue)
     /*if (currentColor == nextColor) {
      if (currentColor == BLUE)
        nextColor = PURPLE;
      if (currentColor == PURPLE)
        nextColor = BLUE;
     }*/
     digitalWrite(LED_base + nextColor, HIGH);
     //millisecondsReverse(3333);
     millisecondsReverse(3000);
   }

}
// ============ END CONTROL CODE ============

// ============ BEGIN LINE FOLLOWER CODE ============
// this method will attempt to follow a line for ~duration~ milliseconds
void linefollow_duration(unsigned long input_duration) {
  unsigned long duration = input_duration;
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
  turnLEDS(LOW);
  Brake();
}


void linefollow_until_wall_dist(int target_dist) {
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

  while (distance_calc() > target_dist) {
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
  turnLEDS(LOW);
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
  digitalWrite(LED_base, HIGH);
  flashLEDs(3);
  digitalWrite(LED_base, LOW);
  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < LF_CALIB_STEPS; i++) {
    showProgressBar(i, LF_CALIB_STEPS);
    updateLEDS(i, LF_CALIB_STEPS);
    qtr.calibrate();
    customDelay(10);
  }

  Serial.println("Calibration for Black Complete!");
  Serial.println();
  turnLEDS(LOW);
  customDelay(300);

  millisecondsForward(400);
  pivotDegrees(45, 255);


  Serial.println("Calibrating for White");
  flashLEDs(3);
  Serial.println("Calibration in Progress : ");
  for(int i = 0; i < LF_CALIB_STEPS; i++) {
    showProgressBar(i, LF_CALIB_STEPS);
    updateLEDS(i, LF_CALIB_STEPS);
    qtr.calibrate();
    customDelay(20);
  }

  turnLEDS(LOW);

  Serial.println("Calibration for White Complete!");
  Serial.println();
  pivotDegrees(-45, 255);
  millisecondsReverse(400);
}

// simple route
void route(){
  cmForward(20);
  customDelay(1000);
  cmReverse(20);
  customDelay(1000);
}
// ============ END LINE FOLLOWER CODE ============

// ============ BEGIN MOTOR CODE ============
void turn_and_hit_coins() {
  // knock coins
  pivotDegrees(-90,255);
  // back up into coins
  millisecondsReverse(350);
  // come back
  millisecondsForward(350);
  // face the right way
  pivotDegrees(90,255);
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

void millisecondsForward(int x) {
  unsigned long currentTime = millis();
  Forward();
  digitalWrite(LED_base, HIGH);
  while ((millis() - currentTime) < x) {
    customDelay(1);
  }
  digitalWrite(LED_base, LOW);
  Brake();
}

void millisecondsReverse(int x) {
  unsigned long starttime = millis();
  Reverse();
  digitalWrite(LED_base+3, HIGH);
  while ((millis() - starttime) < x) {
    customDelay(1);
  }
  digitalWrite(LED_base+3, LOW);
  Brake();
}

void Forward(){
  // motors(HIGH, HIGH, LOW, HIGH, LOW, HIGH);
  int speed = 72;
  //Left Motor
  l_motor(HIGH, HIGH, LOW, speed * 1.35);

  //Right Motor
  r_motor(HIGH, LOW, HIGH, speed);
}

void Reverse(){
  // motors(HIGH, LOW, HIGH, HIGH, HIGH, LOW);
  int speed = 72;
  //Left Motor
  l_motor(HIGH, LOW, HIGH, speed * 1.35);

  //Right Motor
  r_motor(HIGH, HIGH, LOW, speed);
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

  int speed = 72;
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

void pivotDegrees(int degree, int speed) {
  unsigned long duration = int(abs(degree) * 8.8);
  unsigned long start = millis();
  if (degree > 0) {
    while ((millis() - start) < duration) {
      motors(HIGH, LOW, HIGH, HIGH, LOW, HIGH);
      l_motor(HIGH, LOW, HIGH, speed);
      r_motor(HIGH, LOW, HIGH, speed);
    }
    Brake();
  } else {
    while ((millis() - start) < duration) {
      motors(HIGH, HIGH, LOW, HIGH, HIGH, LOW);
      l_motor(HIGH, HIGH, LOW, speed);
      r_motor(HIGH, HIGH, LOW, speed);
    }
    Brake();
  }
} 

void whack_mole() {
  // wacks at a max distance of 10cm away
  // hammer down
  for(int pos = SERVO_STARTING_POINT; pos <= SERVO_ENDING_POINT; pos += 20) {
    myservo.write(pos);
    customDelay(30);
  }
  customDelay(100);
  // hammer up
  for(int pos = SERVO_ENDING_POINT; pos >= SERVO_STARTING_POINT; pos -= 20) {
    myservo.write(pos);
    customDelay(30);
  }
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
    customDelay(100);
    turnLEDS(LOW);
    customDelay(100);
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
      customDelay(100); // Collect data with a small customDelay
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
  uint16_t newR = 0, newG = 0, newB = 0, newC = 0; 


  for(int i = 0; i < 5; i++){
    tcs.getRawData(&r, &g, &b, &c);
    newR += r;
    newG += g;
    newB += b;
    newC += c;
  }

    newR = newR / 5;
    newG = newG / 5;
    newB = newB / 5;
    newC = newC / 5;
    colorTemp = tcs.calculateColorTemperature_dn40(newR, newG, newB, newC);
    lux = tcs.calculateLux(newR, newG, newB);


  if (newB < 5000) {
        if (newR < 3000) {
            return GREEN;
        } else {
            if (lux < 638) {
                return RED;
            } else {
                return YELLOW;
            }
        }
  } else {
      
        if (newR < 2000) {
            return BLUE;
        } else {
            if (lux < 0) {
                return PURPLE;
            } else {
                return WHITE;
            }
        }
      /*
      if (newR >= 2000 && lux > 0) {
        return WHITE;
      } else {
        if ()
      }*/
  }
}

MOLE_COLORS see_the_color() {
  uint16_t r, g, b, c, colorTemp, lux;
  uint16_t newR = 0, newG = 0, newB = 0, newC = 0; 

  for(int i = 0; i < NUM_COLOR_SAMPLES; i++){
    tcs.getRawData(&r, &g, &b, &c);
    newR += r;
    newG += g;
    newB += b;
    newC += c;
  }

  r = newR / NUM_COLOR_SAMPLES;
  g = newG / NUM_COLOR_SAMPLES;
  b = newB / NUM_COLOR_SAMPLES;
  c = newC / NUM_COLOR_SAMPLES;
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(newR, newG, newB);

  // tolerance values
  int r_tol = 1000;
  int g_tol = 1000;
  int b_tol = 1000;
  int c_tol = 1000;

  // blue
  if (r >= (325 - r_tol) && r <= (491 + r_tol) &&
      g >= (1553 - g_tol) && g <= (1714 + g_tol) &&
      b >= (4691 - b_tol) && b <= (4845 + b_tol) &&
      c >= (6446 - c_tol) && c <= (6987 + c_tol)) {
      return BLUE;
  }

  // green
  if (r >= (835 - r_tol) && r <= (849 + r_tol) &&
      g >= (3419 - g_tol) && g <= (3433 + g_tol) &&
      b >= (1338 - b_tol) && b <= (1347 + b_tol) &&
      c >= (5683 - c_tol) && c <= (5724 + c_tol)) {
      return GREEN;
  }

  // purple
  if (r >= (1758 - r_tol) && r <= (1774 + r_tol) &&
      g >= (1763 - g_tol) && g <= (1773 + g_tol) &&
      b >= (4936 - b_tol) && b <= (4970 + b_tol) &&
      c >= (8281 - c_tol) && c <= (8331 + c_tol)) {
      return PURPLE;
  }

  // red
  if (r >= (2997 - r_tol) && r <= (3061 + r_tol) &&
      g >= (434 - g_tol) && g <= (487 + g_tol) &&
      b >= (439 - b_tol) && b <= (470 + b_tol) &&
      c >= (3739 - c_tol) && c <= (3901 + c_tol)) {
      return RED;
  }

  // white
  if (r >= (3385 - r_tol) && r <= (3419 + r_tol) &&
      g >= (4853 - g_tol) && g <= (4869 + g_tol) &&
      b >= (6080 - b_tol) && b <= (6101 + b_tol) &&
      c >= (14096 - c_tol) && c <= (14166 + c_tol)) {
      return WHITE;
  }

  // Yellow
  if (r >= (2002 - r_tol) && r <= (3215 + r_tol) &&
      g >= (1598 - g_tol) && g <= (2019 + g_tol) &&
      b >= (1013 - b_tol) && b <= (1716 + b_tol) &&
      c >= (5244 - c_tol) && c <= (6140 + c_tol)) {
      return YELLOW;
  }

  return BUTTON;
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

// ============ BEGIN WHAC A MOLE CODE ============

void whacAMole(int curMole, int nextMole) {
  int degrees; 
  if(curMole > 2){
    if(nextMole > 2)
      degrees = -1 * (MOLE_DEGREES[curMole] - MOLE_DEGREES[nextMole]);
    else
      degrees = MOLE_DEGREES[nextMole] - MOLE_DEGREES[curMole];
  }
  else{
    if(nextMole > 2)
      degrees = MOLE_DEGREES[nextMole] - MOLE_DEGREES[curMole];
    else 
      degrees = -1 * (MOLE_DEGREES[curMole] - MOLE_DEGREES[nextMole]);
  }

  pivotDegrees(degrees, 255);
}

// ============ END WHAC A MOLE CODE ============

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

int customDelay(unsigned long dur) {
  unsigned long start = millis();
  while(millis() - start < dur) {
  }
}
// ============ END MISC. CODE ============

