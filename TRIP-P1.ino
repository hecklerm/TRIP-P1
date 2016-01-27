/*
 * TRIP-P1 (Positronic/Pi)
 * Code for the Test Robotics IoT Platform (TRIP) version 1.0 (maybe beta!)
 * Copyright (c) 2016 Mark A. Heckler, mark.heckler@gmail.com, @mkheck
 * All rights reserved.
 *
 * This uses example code for the HMC5883 magnetometer/compass.
 * Controls microservo-mounted HC-SR04 proximity sensor.
 *
 * Designed specifically to work with the Adafruit HMC5883 Breakout
 * http://www.adafruit.com/products/1746
 *
 * *** You will also need to install the Adafruit_Sensor library! ***
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <RunningMedian.h>
#include <SPI.h>
#include <dht11.h>
//#include <Versalino.h>

// Define convenience compass directions/bearings
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

// Define convenience directions
#define LEFT -1
#define RIGHT 1

// Define output pins for the L298N (motor control)
#define RT_FWD_PIN 4
#define RT_BWD_PIN 5
#define LF_FWD_PIN 6
#define LF_BWD_PIN 7

// Define the pins we use on the Arduino to send (Trigger) & receive (Echo)
#define PING_PIN 12
#define ECHO_PIN 13

// Define the servo control pin, directions
#define SERVO_PIN 11
#define SERVO_RIGHT 0
#define SERVO_FWDRT 45
#define SERVO_FORWARD 90
#define SERVO_FWDLT 135
#define SERVO_LEFT 180

#define DHT_PIN 2
#define GEIGER_PIN 3

// Assign a unique ID to the magnetometer/compass at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Servo will control HCSR04 proximity sensor
// 12 servo objects can be created on most boards
Servo navServo;
int servoPos = SERVO_FORWARD;
long cm_to_obs = 0;
long cmdNumber;

char incoming[12];

RunningMedian bearings = RunningMedian(10);

// Temp/humidity sensor
dht11 DHT11;
int chk;
String msg;

// Geiger counter
volatile unsigned long radCtr = 0; // GM Tube events
unsigned long cpm = 0;             // radCtr Per Minute
unsigned long curMs;
unsigned long preMs; 

void geiger_pulse() { // Captures count of events from Geiger counter board
  radCtr++;
}

void setup() {
  pinMode(PING_PIN, OUTPUT); // Set up ping pin...
  pinMode(ECHO_PIN, INPUT);  // ...and echo pin
  
  // Servo attached to pin 11
  navServo.attach(SERVO_PIN);
  positionRadar(SERVO_FORWARD);
  
  // Setting the pins as OUTPUT
  pinMode(RT_FWD_PIN, OUTPUT);
  pinMode(RT_BWD_PIN, OUTPUT);
  pinMode(LF_FWD_PIN, OUTPUT);
  pinMode(LF_BWD_PIN, OUTPUT);
  Serial.begin(9600);

  // Initialize the HMC5883 compass sensor
  if(!mag.begin())
  {
    // Problem detecting the HMC5883
    Serial.println("No HMC5883 detected, please check connections.");
    while(1);
  }

  DHT11.attach(DHT_PIN);            // Attach temp/humidity sensor to pin

  pinMode(GEIGER_PIN, INPUT);       // Set pin to input for capturing GM Tube events
  interrupts();                     // Enable interrupts (in case they were previously disabled)
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), geiger_pulse, FALLING);
  preMs = millis();
  
  //Serial.setTimeout(500);   // MAH: Default is 1000ms/1s
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytesUntil('\n', incoming, sizeof(incoming));

    switch (incoming[0]) {
    case 'F':
      forward();
      Serial.println(">F:");
      break;
    case 'B':
      backward();
      Serial.println(">B:");
      break;
    case 'L':
      left();
      Serial.println(">L:");
      break;
    case 'R':
      right();
      Serial.println(">R:");
      break;
    case 'f':
      cmdNumber = pingForward();
      Serial.print(">f:"); Serial.println(cmdNumber);
      break;
    case 'l':
      cmdNumber = pingLeft();
      Serial.print(">l:"); Serial.println(cmdNumber);
      break;
    case 'r':
      cmdNumber = pingRight();
      Serial.print(">r:"); Serial.println(cmdNumber);
      break;
    case 'c':
      // Compass test
      Serial.print("Compass reading="); Serial.println(getCurrentHeading());
      break;
    case 'x':
      // Test case: servo center
      positionRadar(SERVO_FORWARD);
      Serial.println("Centered.");
      break;
    case 'y':
      // Test case: servo left
      positionRadar(SERVO_LEFT);
      Serial.println("Left.");
      break;
    case 'z':
      // Test case: servo right
      positionRadar(SERVO_RIGHT);
      Serial.println("Right.");
      break;
    default:  // STOP!
      stop(0);
      Serial.println(">S:0");
      break;
    }
    
    // Clear the array before looping for more
    for (int i=0; i<sizeof(incoming); i++) {
      incoming[i] = (char)0;
    }
  } else {
    curMs = millis();
    if (curMs - preMs > 999) { // Send data ~1x/sec
      // Temp/humidity
      chk = DHT11.read();
  
      // Geiger reading
      cpm = radCtr * (60000 / (curMs - preMs));
      radCtr = 0;
  
      // MAH TODO: Need to add in the x/y/z readings next
      
      // Build the message to transmit
      msg = "{";
      msg += DHT11.humidity * 100;
      msg += ",";
      msg += DHT11.temperature * 100;
      msg += ",";
      msg += cpm;
      msg += "}";
      Serial.println(msg);

      preMs = curMs;
    }
  }
}

/*
 * PROXIMITY SENSOR METHODS
 */
long pingForward() {
  positionRadar(SERVO_FORWARD, true);  // Set fastPos parameter to true for fast sweep, no 500ms delay
  return ping();
}

long pingLeft() {
  positionRadar(SERVO_LEFT);
  return ping();
}

long pingRight() {
  positionRadar(SERVO_RIGHT);
  return ping();
}

long pingDir(int dir) {
  positionRadar(dir, true);
  return ping();
}

long ping() {
  long duration, cm;
  
  // The ping is triggered by a HIGH pulse of 2 or more microseconds.
  // Send a short LOW pulse beforehand to ensure a clean HIGH pulse.
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  // Now read the signal from the echo pin: a HIGH pulse whose duration is 
  // the time (in microseconds) from the sending of the ping to the reception 
  // of its echo off of an object.
  duration = pulseIn(ECHO_PIN, HIGH);

  // Convert time to distance
  cm = microsecondsToCentimeters(duration);
 
  //Serial.print(cm); Serial.println("cm");
  
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object, we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void positionRadar(int bearing) {
  positionRadar(bearing, false);
}

void positionRadar(int bearing, boolean fastPos) {
  navServo.write(bearing);
  if (fastPos) {
    delay(100);
  } else {
    delay(500);
  }
}

/*
 * NAVIGATION METHODS
 */
// Get the current heading
float getCurrentHeading() {
    /* Get a new sensor event */ 
  sensors_event_t event; 
 
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // MAH: STL magnetic declination is only -1* 17' W, or just over 1 degree.
  // 1 deg = pi / 180 degrees = radians. For -1* 17', it's approximately .017
  // or .02 radians rounded.
  float declinationAngle = 0.02;
  float heading;

  bearings.clear();
  for (int i=0; i<10; i++) { 
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    mag.getEvent(&event);
    heading = atan2(event.magnetic.y, event.magnetic.x) + declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
      
    bearings.add(heading * 180/M_PI);
    
    //Serial.print("Reading "); Serial.print(i); Serial.print(" is "); Serial.print(heading * 180/M_PI); Serial.println(" degrees.");
  }
   
  // Convert radians to degrees for readability.
  //float headingDegrees = heading * 180/M_PI;  MAH
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  //return headingDegrees;
  bearings.sort();
  //Serial.print("Average reading is "); Serial.print(bearings.getAverage()); Serial.println(" degrees.");
  return bearings.getAverage();
}

// Go forward by setting the forward pins to HIGH
void forward() {
  //Serial.println("Forward");
  
  digitalWrite(RT_FWD_PIN, HIGH);
  digitalWrite(RT_BWD_PIN, LOW);
  digitalWrite(LF_FWD_PIN, HIGH);
  digitalWrite(LF_BWD_PIN, LOW);  
}

//Setting the wheels to go backward by setting the backward pins to HIGH
void backward() {
  //Serial.println("Backward");

  digitalWrite(RT_FWD_PIN, LOW);
  digitalWrite(RT_BWD_PIN, HIGH);
  digitalWrite(LF_FWD_PIN, LOW);
  digitalWrite(LF_BWD_PIN, HIGH);
}

void right() {
    pulse(RIGHT);
}

void left() {
    pulse(LEFT);
}

long stop(long duration) {
  digitalWrite(RT_FWD_PIN, LOW);
  digitalWrite(RT_BWD_PIN, LOW);
  digitalWrite(LF_FWD_PIN, LOW);
  digitalWrite(LF_BWD_PIN, LOW);
  delay(duration);
  
  return duration;
}

void pulse(int direction) {
  if (direction == LEFT) {
    digitalWrite(RT_FWD_PIN, HIGH);
    digitalWrite(RT_BWD_PIN, LOW);
    digitalWrite(LF_FWD_PIN, LOW);
    digitalWrite(LF_BWD_PIN, HIGH);
  } else {
    // (direction == RIGHT)
    digitalWrite(RT_FWD_PIN, LOW);
    digitalWrite(RT_BWD_PIN, HIGH);
    digitalWrite(LF_FWD_PIN, HIGH);
    digitalWrite(LF_BWD_PIN, LOW);
  }
  delay(350);
  stop(500);
}

