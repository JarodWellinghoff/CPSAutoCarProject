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
#include <protothreads.h>

#define SLOW_DIST 40.0  // Distance from an object when the car should slow down
#define STOP_DIST 20.0  // Distance from an object when the car should stop
#define ARRIVED -50
#define WINDOW_SIZE 5
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
#define BUTTON_PIN 4

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
int dist_F,
  dist_B,
  dist_L,
  dist_R,
  rssi;

bool loopState = false,
     IRS_F,
     IRS_B;

int USS_array[] = { USS_F_PIN, USS_B_PIN, USS_L_PIN, USS_R_PIN };  // Array of the ultrasonic sensor pins
int distance_array[] = { dist_F, dist_B, dist_L, dist_R };         // Array of distances from ultrasonic sensors
int IRS_array[] = { IRS_F_PIN, IRS_B_PIN };
bool IRS_status_array[] = { IRS_F, IRS_B };

SoftwareSerial BT_Serial(2, 3);  // (RX, TX), Bluetooth module pins

pt ptRSSI;
int RSSIThread(struct pt* pt) {
  PT_BEGIN(pt);

  for(;;) {
  BT_Serial.print("AT+RSSI?");
  PT_SLEEP(pt, 150);

  if (BT_Serial.available()) {
    str = "";

    prevMillis = millis();
    while (millis() - prevMillis < READ_TIME) {
      if (BT_Serial.available()) {
        str += (char)BT_Serial.read();
      }
    }
    // Serial.println(str);
    rssi = str.substring(7).toInt();
  }
  }
  PT_END(pt);
}

pt ptGetDistance;
int GetDistanceThread(struct pt* pt) {
  PT_BEGIN(pt);

  for(;;) {
    pinMode(USS_F_PIN, OUTPUT);
    digitalWrite(USS_F_PIN, LOW);
    PT_SLEEP(pt, 2);
    digitalWrite(USS_F_PIN, HIGH);
    PT_SLEEP(pt, 3);
    digitalWrite(USS_F_PIN, LOW);


    pinMode(USS_F_PIN, INPUT);
    f_duration = pulseIn(USS_F_PIN, HIGH);
    f_distance = f_duration * 0.01715;

    dist_F_samples.add(f_distance);
    dist_F = dist_F_samples.getAverage();

    pinMode(USS_B_PIN, OUTPUT);
    digitalWrite(USS_B_PIN, LOW);
    PT_SLEEP(pt, 2);
    digitalWrite(USS_B_PIN, HIGH);
    PT_SLEEP(pt, 3);
    digitalWrite(USS_B_PIN, LOW);


    pinMode(USS_B_PIN, INPUT);
    b_duration = pulseIn(USS_B_PIN, HIGH);
    b_distance = b_duration * 0.01715;

    dist_B_samples.add(b_distance);
    dist_B = dist_B_samples.getAverage();

    pinMode(USS_L_PIN, OUTPUT);
    digitalWrite(USS_L_PIN, LOW);
    PT_SLEEP(pt, 2);
    digitalWrite(USS_L_PIN, HIGH);
    PT_SLEEP(pt, 3);
    digitalWrite(USS_L_PIN, LOW);


    pinMode(USS_L_PIN, INPUT);
    l_duration = pulseIn(USS_L_PIN, HIGH);
    l_distance = l_duration * 0.01715;

    dist_L_samples.add(l_distance);
    dist_L = dist_L_samples.getAverage();

    pinMode(USS_R_PIN, OUTPUT);
    digitalWrite(USS_R_PIN, LOW);
    PT_SLEEP(pt, 2);
    digitalWrite(USS_R_PIN, HIGH);
    PT_SLEEP(pt, 3);
    digitalWrite(USS_R_PIN, LOW);


    pinMode(USS_R_PIN, INPUT);
    r_duration = pulseIn(USS_R_PIN, HIGH);
    r_distance = r_duration * 0.01715;

    dist_R_samples.add(r_distance);
    dist_R = dist_R_samples.getAverage();

    // Serial.println(dist_F);
    // Serial.println(dist_B);
    // Serial.println(dist_L);
    // Serial.println(dist_L);
  }

  PT_END(pt);
}

pt ptIRS;
int IRSThread(struct pt* pt) {
  PT_BEGIN(pt);

  for(;;) {
    IRS_F = digitalRead(IRS_F_PIN);
    IRS_B = digitalRead(IRS_B_PIN);

    // Serial.println(IRS_F);
    // Serial.println(IRS_B);
  }
  PT_END(pt);
}

pt ptMotorControl;
int MotorControlThread(struct pt* pt) {
  PT_BEGIN(pt);

  for(;;){
    if (dist_F > SLOW_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(255);
    } else if (SLOW_DIST >= dist_F && dist_F > STOP_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(200);
    }
  }

  PT_END(pt);
}

void setup() {
  PT_INIT(&ptRSSI);
  PT_INIT(&ptGetDistance);
  PT_INIT(&ptIRS);
  PT_INIT(&ptMotorControl);
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
  PT_SCHEDULE(RSSIThread(&ptRSSI));
  PT_SCHEDULE(GetDistanceThread(&ptGetDistance));
  PT_SCHEDULE(IRSThread(&ptIRS));
  // Loop will run when toogle switch is set to true
  if (loopState == true) {

  PT_SCHEDULE(RSSIThread(&ptRSSI));
  PT_SCHEDULE(GetDistanceThread(&ptGetDistance));
  PT_SCHEDULE(IRSThread(&ptIRS));
  PT_SCHEDULE(MotorControlThread(&ptMotorControl));

    // Get statuses from IR sensors
    // for (int i = 0; i < 2; i++) {
      // IRS_status_array[i] = digitalRead(IRS_array[i]);
    // }

    // Get distances from ultrasonic sensors
    // for (int i = 0; i < 4; i++) {
      // distance_array[i] = getDistance(USS_array[i]);
      //      if (i == 3)
      //        Serial.println(distance_array[i]);
      //      else {
      //        Serial.print(distance_array[i]); Serial.print(",");
      //      }
    // }

    //     if (!IRS_status) {
    //       driveMotor->run(BACKWARD);
    //       driveMotor->setSpeed(255);
    //       driveMotor->setSpeed(1);

    // //      driveMotor->run(RELEASE);
    //     }
    if (dist_F > SLOW_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(255);
    } else if (SLOW_DIST >= dist_F && dist_F > STOP_DIST) {
      driveMotor->run(FORWARD);
      driveMotor->setSpeed(200);
    }
    //    } else if (STOP_DIST >= dist_F) {
    //      driveMotor->run(BACKWARD);
    //      driveMotor->setSpeed(10);

    //  delay(1000);
  }
}

// int getRSSI() {
//   BT_Serial.print("AT+RSSI?");
//   delay(150);

//   if (BT_Serial.available()) {
//     str = "";

//     prevMillis = millis();
//     while (millis() - prevMillis < READ_TIME) {
//       if (BT_Serial.available()) {
//         str += (char) BT_Serial.read();
//       }
//     }
//     Serial.println(str);
//     return str.substring(7).toInt();
//   }

// }

// int getDistance(int USSPin) {
//   pinMode(USSPin, OUTPUT);
//   digitalWrite(USSPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(USSPin, HIGH);
//   delayMicroseconds(3);
//   digitalWrite(USSPin, LOW);


//   pinMode(USSPin, INPUT);
//   duration = pulseIn(USSPin, HIGH);
//   distance = duration * 0.01715;

//   if (distance > 400)
//     distance = 400;
//   else
//     distance = distance;

//   switch (USSPin) {
//     case 13:
//       dist_F_samples.add(distance);
//       dist_F = dist_F_samples.getAverage();
//       return dist_F;
//     case 12:
//       dist_B_samples.add(distance);
//       dist_B = dist_B_samples.getAverage();
//       return dist_B;
//     case 11:
//       dist_L_samples.add(distance);
//       dist_L = dist_L_samples.getAverage();
//       return dist_L;
//     case 10:
//       dist_R_samples.add(distance);
//       dist_R = dist_R_samples.getAverage();
//       return dist_R;
//   }
// }
