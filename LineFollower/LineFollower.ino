// Line follower robot
// Start from the RIGHT side of the board, for the encoderCountGoal
// to be correct, so the robot stops on top of the ramp.

#include <AFMotor.h>
#include <SoftwareSerial.h>

// #define DEBUG           1

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define RightMotorPin   3   // Right motor is connected to this pin
#define LeftMotorPin    4   // Left motor is connected to this pin
#define leftEncoderPin  2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt)
#define rightEncoderPin 3
#define SPEED           250 // Set speed to be used for motors

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

int fastMotorSpeed;
int slowMotorSpeed;

// Encoder variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
unsigned int encoderCountGoal = 405;

// Light sensor:
const int lightSensorPin = A3;
unsigned int lightSensorValue;

// IR sensors:
const int analogInPinShort = A2; // Analog input from short range reflector
unsigned int sensorValueShort;   // Short range sensor value

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

  // Setup encoder:
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
}

void loop() {
  // Read short range sensor value
  sensorValueShort = analogRead(analogInPinShort);
  if (sensorValueShort > 500)
  {
    Left_Motor.run(BACKWARD);
    Right_Motor.run(RELEASE);
    delay(30);

    SharpTurnLeft();
    delay(50);
  }

  // Read IR sensor values
  lightSensorValue = analogRead(lightSensorPin);
  if (lightSensorValue < 350)
  {
    // To the right of the line - turn left
    TurnRight();
  }
  else
  {
    // Over the line - turn right
    TurnLeft();
  }

  printDebug();

  if (rightEncoderCount > encoderCountGoal)
  {
    StopMotors();
    while(true){}
  }
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

void TurnLeft()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(RELEASE);
}

void TurnRight()
{
  Left_Motor.run(FORWARD);
  Right_Motor.run(RELEASE);
}

void SharpTurnLeft()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void SharpTurnRight()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void DriveForward()
{
  //For rear-wheel drive run it BACKWARD
  //For FWD run forward
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void StopMotors()
{
  Right_Motor.run(RELEASE);
  Left_Motor.run(RELEASE);
}

void printDebug()
{
  #ifdef DEBUG
  Serial.print("LightSensor = " );
  Serial.println(lightSensorValue);

  Serial.print("RangeSensor = " );
  Serial.println(sensorValueShort);

  Serial.print("RightEncoderCount = ");
  Serial.println(rightEncoderCount);

  delay(500);
  #endif
}
