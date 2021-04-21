// --------------------------------------------------------------------------------
// AUTHOR: Jarod Wellinghoff
// FILENAME: Self_Driving_Car.ino
// SPECIFICATION: Obstacle avoiding, autonomus car
// FOR: CS 4331 Cyber Physical Systems Section 007

#include <SoftwareSerial.h>
#include <ezButton.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <RunningMedian.h>

#define IBEACON_MAC DD330A111AF5  // Mac address of the iBeacon
#define SLOW_DIST 40.0 // Distance from an object when the car should slow down
#define STOP_DIST 20.0 // Distance from an object when the car should stop
#define WINDOW_SIZE 10
#define RIGHT FORWARD
#define LEFT BACKWARD

// Ultrasonic Sensor pins
#define USS_F 13
#define USS_B 12
#define USS_L 11
#define USS_R 10

// Infrared Senor pins
#define IRS 9

// Button pin
#define buttonPin 4

// Create DC motor objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *driveMotor = AFMS.getMotor(1);
Adafruit_DCMotor *turnMotor = AFMS.getMotor(2);

// Create ezButton object
ezButton button(buttonPin);

// Create Running Median objects
RunningMedian dist_F_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_B_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_L_samples = RunningMedian(WINDOW_SIZE);
RunningMedian dist_R_samples = RunningMedian(WINDOW_SIZE);

float duration, // The time for the ultrasonic sensor to send and recieve a signal
      distance; // The distance given by the ultrasonic sensor
int   dist_F,
      dist_B,
      dist_L,
      dist_R;

int USS_array[] = {USS_F, USS_B, USS_L, USS_R}; // Array of the ultrasonic sensor pins
int distance_array[] = {dist_F, dist_B, dist_L, dist_R};  // Array of distances from ultrasonic sensors

bool loopState = false,
     IRS_status;

char c = ' ';
bool NL = true;
char DISIstr[100];
SoftwareSerial BT_Serial(3, 2); // (RX, TX), Bluetooth module pins

void setup() {
  // Set the Serial Monotor
  Serial.begin(9600);
  Serial.print("Sketch: "); Serial.println(__FILE__);
  Serial.print("Uploaded: "); Serial.print(__DATE__); Serial.print(" @ "); Serial.println(__TIME__);

  // Set the Bluetooth Serial
  BT_Serial.begin(9600);
  Serial.print(BT_Serial.read());
  Serial.println("BTserial started at 9600");

  // Activate motors
  AFMS.begin();
  driveMotor->setSpeed(200);
  turnMotor->setSpeed(200);
  driveMotor->run(RELEASE);
  turnMotor->run(RELEASE);

  // Set up infrared sensors
  pinMode(IRS, INPUT);

  button.setDebounceTime(50); // Set debounce time to 50 milliseconds
  delay(1000);
}

void loop() {
  button.loop(); // Calls the loop function

  // Changes loopState to !loopState everytime the button is pressed
  if (button.isPressed()) {
    driveMotor->run(RELEASE);
    turnMotor->run(RELEASE);
    if (loopState == false)
      loopState = true;
    else
      loopState = false;
  }

  // Loop will run when toogle switch is set to true
  if (loopState == true) {
    //  BT_Serial.print("AT+DISI?");
    //
    //  // Read from the Bluetooth module and send to the Arduino Serial Monitor
    //  while (BT_Serial.available())
    //  {
    //    c = BT_Serial.read();
    //    Serial.write(c);
    //    delay(10);
    //
    //  }
    //
    //
    //  // Read from the Serial Monitor and send to the Bluetooth module
    //  if (Serial.available()) {
    //    c = Serial.read();
    //    if (c != 10 & c != 13) {
    //      BT_Serial.write(c);
    //    }
    //    if (NL) {
    //      Serial.print("\r\n>");
    //      NL = false;
    //    }
    //    Serial.write(c);
    //    if (c == 10) {
    //      NL = true;
    //    }
    //  }



    // Get statuses from IR sensors
    IRS_status = digitalRead(IRS);

    //    Serial.print(IRS_F0_status); Serial.print(IRS_F1_status); Serial.print(IRS_B0_status); Serial.println(IRS_B1_status);

    // Get distances from ultrasonic sensors
    for (int i = 0; i < 4; i++) {
      distance_array[i] = getDistance(USS_array[i]);
      //      if (i == 3)
      //        Serial.println(distance_array[i]);
      //      else {
      //        Serial.print(distance_array[i]); Serial.print(",");
      //      }
    }

    if (!IRS_status) {
      driveMotor->run(BACKWARD);
      driveMotor->setSpeed(255);
      driveMotor->setSpeed(1);
      
//      driveMotor->run(RELEASE);
    } else if (dist_F > SLOW_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(255);
    } else if (SLOW_DIST >= dist_F && dist_F > STOP_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(200);}
//    } else if (STOP_DIST >= dist_F) {
//      driveMotor->run(BACKWARD);
//      driveMotor->setSpeed(10);
    
    //  delay(1000);
  }
}

int getDistance(int USSPin) {
  pinMode(USSPin, OUTPUT);
  digitalWrite(USSPin, LOW);
  delayMicroseconds(2);
  digitalWrite(USSPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(USSPin, LOW);


  pinMode(USSPin, INPUT);
  duration = pulseIn(USSPin, HIGH);
  distance = duration * 0.01715;

  if (distance > 400)
    distance = 400;
  else
    distance = distance;

  switch (USSPin) {
    case 13:
      dist_F_samples.add(distance);
      dist_F = dist_F_samples.getAverage();
      return dist_F;
    case 12:
      dist_B_samples.add(distance);
      dist_B = dist_B_samples.getAverage();
      return dist_B;
    case 11:
      dist_L_samples.add(distance);
      dist_L = dist_L_samples.getAverage();
      return dist_L;
    case 10:
      dist_R_samples.add(distance);
      dist_R = dist_R_samples.getAverage();
      return dist_R;
  }
}
