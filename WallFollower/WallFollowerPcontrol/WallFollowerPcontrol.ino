// Counter-clockwise wall follower robot

#include <AFMotor.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <math.h>

#define DEBUG           1

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define RightMotorPin   3   // Right motor is connected to this pin
#define LeftMotorPin    4   // Left motor is connected to this pin
#define leftEncoderPin  2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt)
#define rightEncoderPin 3
#define SPEED           180 // Set speed to be used for motors

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

// P controller control variables
double input, output;
int desiredSensorValue = 145;
double scaledOutput    = 1;
int fastMotorSpeed     = 150;
int slowMotorSpeed     = 150;
float Kp = 2.2;

int r = 60; //Set distance r away from wall in mm
int A = 869.9; //constants for fit function
int b = -0.1316;


// Encoder variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int encoderCountGoal = 290;

// IR sensors:
const int analogInPinShort = A0; // Analog input from short range reflector
const int analogInPinLong = A1;  // Analog input from long range reflector

int sensorValueShort = 0;     // Short range sensor value
int sensorValueLong = 0;      // Long range sensor value

// Photoresistant sensor:
const int lightSensorPin = A3;
int lightSensorThreshold = 200;

// LCD Screen:
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);
  mySerial.print("?x00?y1");
  mySerial.print("?f");
  mySerial.print("?l");

  // Set motor speed:
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  // Set light sensor:
  pinMode(lightSensorPin, INPUT);
  digitalWrite(lightSensorPin, HIGH);

  // Setup hardware interrupt:
  int leftInterruptPin = leftEncoderPin - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt(leftInterruptPin, incrementLeftEncoder, FALLING);   // Attach interrupt pin, name of function to be called
  // During interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  int rightInterruptPin = rightEncoderPin - 2;
  attachInterrupt(rightInterruptPin, incrementRightEncoder, FALLING);

  // Setup encoder i.e. break-beam:
  pinMode(leftEncoderPin, INPUT);
  digitalWrite(leftEncoderPin, HIGH);
  pinMode(rightEncoderPin, INPUT);
  digitalWrite(rightEncoderPin, HIGH);


  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);
  /*while (analogRead(lightSensorPin) <= lightSensorThreshold)
  {
    // Wait until switch is pressed.
    delay(150);
  }*/
  while (digitalRead(switchPin))
  {
    // Wait until switch is pressed.
    delay(150);
  }
}

long int lastPrint = 0;

void loop() {
  //Serial.println(analogRead(lightSensorPin));

  //mySerial.print(analogRead(lightSensorPin));
  // Read IR sensor values
  sensorValueShort = analogRead(analogInPinShort);
  int dist = floor(log(sensorValueShort/A)/b);
  int diff = dist - r;
  float theta = Kp*diff;
  int error = desiredSensorValue - sensorValueShort;
  float Ke = Kp * error;
  int rightSpeed = constrain(SPEED+int(theta), 0, 255);
  int leftSpeed = constrain(SPEED-int(theta), 0, 255);

  if(millis() - lastPrint > 200) {
    mySerial.print("?f");
    mySerial.print("?x00?y0");
    mySerial.print("R: ");
    mySerial.print(rightSpeed);
    mySerial.print("?x00?y1");
    mySerial.print("L: ");
    mySerial.print(leftSpeed);
    lastPrint = millis();
  }

  Right_Motor.setSpeed(rightSpeed);
  Left_Motor.setSpeed(leftSpeed);

  DriveForward();

  if (rightEncoderCount > encoderCountGoal)
  {
    // Stop robot after one lap
    StopMotors();
    while(true);
  }




  //printDebug();

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  //delay(10);
}

// --- //

void incrementLeftEncoder()
{
  ++leftEncoderCount;
  displayEncoderCounts();
}

void incrementRightEncoder()
{
  ++rightEncoderCount;
  displayEncoderCounts();
}

void displayEncoderCounts()
{
  // mySerial.print("?x00?y1");
  // mySerial.print("L: ");
  // mySerial.print(leftEncoderCount);
  // mySerial.print(" R: ");
  // mySerial.print(rightEncoderCount);
}

void SharpTurnRight()
{

  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
}

void SharpTurnLeft()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void DriveForward()
{
  //For rear-wheel drive run it BACKWARD
  //For FWD run forward
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void StopMotors()
{
  Right_Motor.run(RELEASE);
  Left_Motor.run(RELEASE);
}

void printDebug()
{
  #ifdef DEBUG
  // Serial.print("sensorShort = " );
  // Serial.println(sensorValueShort);
  // //Serial.print(" sensorLong = ");
  // //Serial.println(sensorValueLong);
  //
  // Serial.print("Input: ");
  // Serial.println(input);
  // Serial.print("PID output: ");
  // Serial.println(output);
  // Serial.print("Scaled output: ");
  // Serial.println(scaledOutput);
  //
  // mySerial.print("?x00?y1");
  // mySerial.print("output: ");
  // mySerial.print(scaledOutput);
  #endif
}
