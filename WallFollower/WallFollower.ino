// Counter-clockwise wall follower robot

#include <AFMotor.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

//#define DEBUG           1

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define RightMotorPin   4   // Right motor is connected to this pin
#define LeftMotorPin    3   // Left motor is connected to this pin
#define SPEED           200 // Set speed to be used for motors

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

// P controller control variables
double input, output;
int desiredSensorValue = 300;
double scaledOutput    = 1;
int fastMotorSpeed     = 150;
int slowMotorSpeed     = 150;

// Encoder variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int encoderCountGoal = 10;

// IR sensors:
const int analogInPinShort = A0; // Analog input from short range reflector
const int analogInPinLong = A1;  // Analog input from long range reflector

int sensorValueShort = 0;     // Short range sensor value
int sensorValueLong = 0;      // Long range sensor value

// LCD Screen:
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);

  // Set motor speed:
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // Read IR sensor values
  sensorValueShort = analogRead(analogInPinShort);
  sensorValueLong = analogRead(analogInPinLong);
  input = abs(sensorValueShort - desiredSensorValue);

  slowMotorSpeed = constrain(SPEED - 100, 100, 250);
  fastMotorSpeed = constrain(SPEED + 50, 150, 250);

  Serial.print(slowMotorSpeed);
  Serial.print(" ");
  Serial.println(fastMotorSpeed);

  // Slight turn
  if (sensorValueShort > desiredSensorValue)
  {
    // Too close to the wall - turn left
    Right_Motor.setSpeed(fastMotorSpeed);
    Left_Motor.setSpeed(slowMotorSpeed);
    DriveForward();
  }
  else
  {
    // Too far from the wall - turn right
    Right_Motor.setSpeed(slowMotorSpeed);
    Left_Motor.setSpeed(fastMotorSpeed);
    DriveForward();
  }

  printDebug();

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);
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
  mySerial.print("?x00?y1");
  mySerial.print("L: ");
  mySerial.print(leftEncoderCount);
  mySerial.print(" R: ");
  mySerial.print(rightEncoderCount);
}

void SharpTurnRight()
{

  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void SharpTurnLeft()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
}

void DriveForward()
{
  //For rear-wheel drive run it BACKWARD
  //For FWD run forward
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void printDebug()
{
  #ifdef DEBUG
  Serial.print("sensorShort = " );
  Serial.println(sensorValueShort);
  //Serial.print(" sensorLong = ");
  //Serial.println(sensorValueLong);

  Serial.print("Input: ");
  Serial.println(input);
  Serial.print("PID output: ");
  Serial.println(output);
  Serial.print("Scaled output: ");
  Serial.println(scaledOutput);

  mySerial.print("?x00?y1");
  mySerial.print("output: ");
  mySerial.print(scaledOutput);
  #endif
}
