// Counter-clockwise wall follower robot

#include <AFMotor.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

//#define DEBUG           1

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define RightMotorPin   3   // Right motor is connected to this pin
#define LeftMotorPin    4   // Left motor is connected to this pin
#define leftEncoderPin  2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt)
#define rightEncoderPin 3
#define SPEED           200 // Set speed to be used for motors

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

// P controller control variables
int diff;
int desiredSensorValue = 145;
double scaledOutput    = 1;
int fastMotorSpeed     = 150;
int slowMotorSpeed     = 150;

// Encoder variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int          encoderCountGoal = 300;

// IR sensors:
const int analogInPinShort = A0; // Analog input from short range reflector
const int analogInPinLong  = A1; // Analog input from long range reflector

int sensorValueShort = 0;        // Short range sensor value
int sensorValueLong  = 0;        // Long range sensor value

// Photoresistant sensor:
const int lightSensorPin = A3;
int lightSensorThreshold = 200;

// LCD Screen:
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);

  // Set motor speed:
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

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

  // Set light sensor:
  pinMode(lightSensorPin, INPUT);
  digitalWrite(lightSensorPin, HIGH);

  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // Clear LCD screen
  mySerial.print("?f"); // Send clear screen command to LCD

  while (analogRead(lightSensorPin) > lightSensorThreshold)
  {
    // Wait until switch is pressed.
    delay(150);
  }
}

void loop() {
  // Read IR sensor values
  sensorValueShort = analogRead(analogInPinShort);
  sensorValueLong = analogRead(analogInPinLong);
  diff = abs(sensorValueShort - desiredSensorValue);

  if (abs(sensorValueShort - desiredSensorValue) < 15)
  {
    Right_Motor.setSpeed(SPEED - 50);
    Left_Motor.setSpeed(SPEED - 50);

    DriveForward();
  }
  else if (sensorValueShort > desiredSensorValue)
  {
    slowMotorSpeed = constrain(SPEED, 0, 255);
    fastMotorSpeed = constrain(SPEED, 0, 255);

    // Too close to the wall - turn left
    Right_Motor.setSpeed(fastMotorSpeed);
    Left_Motor.setSpeed(slowMotorSpeed);

    SharpTurnRight();
  }
  else
  {
    // Too far from the wall - turn right
    slowMotorSpeed = constrain(SPEED, 0, 255);
    fastMotorSpeed = constrain(SPEED, 0, 255);

    Right_Motor.setSpeed(slowMotorSpeed);
    Left_Motor.setSpeed(fastMotorSpeed);

    SharpTurnLeft();
  }

  if (rightEncoderCount > encoderCountGoal)
  {
    // Stop robot after one lap
    StopMotors();
    while(true);
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
  Serial.print("sensorShort = " );
  Serial.println(sensorValueShort);
  //Serial.print(" sensorLong = ");
  //Serial.println(sensorValueLong);

  Serial.print("diff: ");
  Serial.println(diff);
  //Serial.print("PID output: ");
  //Serial.println(output);
  //Serial.print("Scaled output: ");
  //Serial.println(scaledOutput);

  //mySerial.print("?x00?y1");
  //mySerial.print("output: ");
  //mySerial.print(scaledOutput);

  delay(500);
  #endif
}
