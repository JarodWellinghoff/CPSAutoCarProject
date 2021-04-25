```cpp
// --------------------------------------------------------------------------------
// AUTHOR: Jarod Wellinghoff
// FILENAME: Self_Driving_Car_Thread.ino
// SPECIFICATION: Obstacle avoiding, autonomus car with protothreading
// FOR: CS 4331 Cyber Physical Systems Section 007

#include <SoftwareSerial.h>
#include <ezButton.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <RunningMedian.h>
#include "protothreads.h"

#define SLOW_DIST 40.0  // Distance from an object when the car should slow down
#define STOP_DIST 20.0  // Distance from an object when the car should stop
#define ARRIVED -60
#define WINDOW_SIZE 2
#define RIGHT FORWARD
#define LEFT BACKWARD

// Ultrasonic Sensor pins
#define USS_F_PIN 13
#define USS_B_PIN 12
#define USS_L_PIN 11
#define USS_R_PIN 10

// Infrared Senor pins
#define IRS_F_PIN 8
#define IRS_B_PIN 9

// Button pin
#define BUTTON_PIN 7

// Create DC motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* driveMotor = AFMS.getMotor(1);
Adafruit_DCMotor* turnMotor = AFMS.getMotor(2);

// Create ezButton object
ezButton button(BUTTON_PIN);

// Create Running Median objects
RunningMedian dist_F_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_B_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_L_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_R_samples = RunningMedian(WINDOW_SIZE);

const int READ_TIME = 500;  //ms

unsigned long prevMillis;

String str = "";

float f_duration,
  duration,
  b_duration,
  l_duration,
  r_duration,
  f_distance,
  b_distance,
  l_distance,
  r_distance;
float dist_F,
  dist_B,
  dist_L,
  dist_R;

bool loopState = false,
     IRS_F,
     IRS_B,
     going_forward = true,
     going_backward = false,
     braking = false,
     arrived = false;

int USS_array[] = { USS_F_PIN, USS_B_PIN, USS_L_PIN, USS_R_PIN };  // Array of the ultrasonic sensor pins
float distance_array[] = { dist_F, dist_B, dist_L, dist_R };       // Array of distances from ultrasonic sensors
int IRS_array[] = { IRS_F_PIN, IRS_B_PIN };
bool IRS_status_array[] = { IRS_F, IRS_B };
int rssi = -1000;

SoftwareSerial BT_Serial(2, 3);  // (RX, TX), Bluetooth module pins

void setup() {
  // Set the Serial Monotor
  Serial.begin(9600);
  Serial.print("Sketch: ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.print(__DATE__);
  Serial.print(" @ ");
  Serial.println(__TIME__);

  // Set the Bluetooth Serial
  BT_Serial.begin(9600);
  Serial.println("BTserial started at 9600");

  // Activate motors
  AFMS.begin();
  driveMotor->setSpeed(200);
  turnMotor->setSpeed(200);
  driveMotor->run(RELEASE);
  turnMotor->run(RELEASE);

  // Set up infrared sensors
  pinMode(IRS_F_PIN, INPUT);
  pinMode(IRS_B_PIN, INPUT);

  button.setDebounceTime(50);  // Set debounce time to 50 milliseconds
  delay(100);
}

void loop() {
  button.loop();  // Calls the loop function

  // Changes loopState to !loopState everytime the button is pressed
  if (button.isPressed()) {
    driveMotor->run(RELEASE);
    turnMotor->run(RELEASE);
    if (loopState == false)
      loopState = true;
    else
      loopState = false;
  }

  // Serial.println(getRSSI());
  // Loop will run when toogle switch is set to true
  if (loopState == true) {
    rssi = getRSSI();
    arrived = haveArrived(rssi);
    Serial.println(arrived);

    if (arrived) {
      goForwardBrake(10);
      goBackwardBrake(10);
      loopState = false;
    }
    dist_F = getForwardDistance();
    IRS_F = getForwardIRS();
    if (dist_F < SLOW_DIST || IRS_F) {
      goForward(100, 100);


    }
  }
}

bool haveArrived(int rssi) {
  if (rssi > ARRIVED)
    return true;
  else
    return false;
}

void changePath() {
  
}

void goBackwardBrake(int duration) {
  going_backward = false;
  braking = true;

  turnMotor->run(RELEASE);

  driveMotor->run(FORWARD);
  driveMotor->setSpeed(255);
  delay(10);
  driveMotor->run(RELEASE);
  delay(duration);
}

void goForwardBrake(int duration) {
  going_forward = false;
  braking = true;

  turnMotor->run(RELEASE);

  driveMotor->run(BACKWARD);
  driveMotor->setSpeed(255);
  delay(10);
  driveMotor->run(RELEASE);
  delay(duration);
}

void goBackwardRight(int sd, int duration) {
  going_forward = false;
  going_backward = true;
  braking = false;

  turnMotor->run(RIGHT);
  turnMotor->setSpeed(255);

  driveMotor->run(BACKWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

void goBackwardLeft(int sd, int duration) {
  going_forward = false;
  going_backward = true;
  braking = false;

  turnMotor->run(LEFT);
  turnMotor->setSpeed(255);

  driveMotor->run(BACKWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

void goForwardRight(int sd, int duration) {
  going_forward = true;
  going_backward = false;
  braking = false;

  turnMotor->run(RIGHT);
  turnMotor->setSpeed(255);

  driveMotor->run(FORWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

void goForwardLeft(int sd, int duration) {
  going_forward = true;
  going_backward = false;
  braking = false;

  turnMotor->run(LEFT);
  turnMotor->setSpeed(255);

  driveMotor->run(FORWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

void goBackward(int sd, int duration) {
  going_forward = false;
  going_backward = true;
  braking = false;

  turnMotor->run(RELEASE);

  driveMotor->run(BACKWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

void goForward(int sd, int duration) {
  going_forward = true;
  going_backward = false;
  braking = false;

  turnMotor->run(RELEASE);

  driveMotor->run(FORWARD);
  driveMotor->setSpeed(sd);
  delay(duration);
}

int getRSSI() {
  BT_Serial.print("AT+RSSI?");
  delay(150);

  if (BT_Serial.available()) {
    str = "";

    prevMillis = millis();
    while (millis() - prevMillis < READ_TIME) {
      if (BT_Serial.available()) {
        str += (char)BT_Serial.read();
      }
    }
    return str.substring(7).toInt();
  }
}

float getForwardDistance() {
  pinMode(USS_F_PIN, OUTPUT);
  digitalWrite(USS_F_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_F_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(USS_F_PIN, LOW);


  pinMode(USS_F_PIN, INPUT);
  f_duration = pulseIn(USS_F_PIN, HIGH);
  f_distance = f_duration * 0.01715;

  dist_F_samples.add(f_distance);
  return dist_F_samples.getAverage();
}

float getBackwardDistance() {
  pinMode(USS_B_PIN, OUTPUT);
  digitalWrite(USS_B_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_B_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(USS_B_PIN, LOW);


  pinMode(USS_B_PIN, INPUT);
  b_duration = pulseIn(USS_B_PIN, HIGH);
  b_distance = b_duration * 0.01715;

  dist_B_samples.add(b_distance);
  return dist_B_samples.getAverage();
}

float getLeftDistance() {
  pinMode(USS_L_PIN, OUTPUT);
  digitalWrite(USS_L_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_L_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(USS_L_PIN, LOW);


  pinMode(USS_L_PIN, INPUT);
  l_duration = pulseIn(USS_L_PIN, HIGH);
  l_distance = l_duration * 0.01715;

  dist_L_samples.add(l_distance);
  return dist_L_samples.getAverage();
}

float getRightDistance() {
  pinMode(USS_R_PIN, OUTPUT);
  digitalWrite(USS_R_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_R_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(USS_R_PIN, LOW);


  pinMode(USS_R_PIN, INPUT);
  r_duration = pulseIn(USS_R_PIN, HIGH);
  r_distance = r_duration * 0.01715;

  dist_R_samples.add(r_distance);
  return dist_R_samples.getAverage();
}

bool getForwardIRS() {
  return !digitalRead(IRS_F_PIN);
}

bool getBackwardIRS() {
  return !digitalRead(IRS_B_PIN);
}
```
