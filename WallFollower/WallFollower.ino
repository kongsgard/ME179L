// Counter-clockwise wall follower robot

#include <AFMotor.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

#define DEBUG           1

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define RightMotorPin   3   // Right motor is connected to this pin
#define LeftMotorPin    4   // Left motor is connected to this pin
#define SPEED           250 // Set speed to be used for motors

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

// PID control variables
double setpoint, input, output;

// Aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

// Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);

// Encoder variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int encoderCountGoal = 10;

// IR sensors:
const int analogInPinShort = A0; // Analog input from short range reflector
const int analogInPinLong = A1;  // Analog input from long range reflector

double sensorValueShort = 0;     // Short range sensor value
double sensorValueLong = 0;      // Long range sensor value

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

  // Set IR short range setpoint to a value corresponding to 6 cm
  setpoint = 300;
}

void loop() {
  // Read IR sensor values
  sensorValueShort = analogRead(analogInPinShort);
  sensorValueLong = analogRead(analogInPinLong);
  input = sensorValueShort;

  setPIDTunings();
  myPID.Compute();

  if (abs(output) < 100)
  {
    // Slight turn
    if (output < 0)
    {
      // Too close to the wall - turn left
      Right_Motor.setSpeed(SPEED);
      Left_Motor.setSpeed(0.6 * SPEED);
      DriveForward();
    }
    else
    {
      // Too far from the wall - turn right
      Right_Motor.setSpeed(0.6 * SPEED);
      Left_Motor.setSpeed(SPEED);
      DriveForward();
    }
  }
  else
  {
    // Sharp turn
    Right_Motor.setSpeed(SPEED);
    Left_Motor.setSpeed(SPEED);
    if (output < 0)
    {
      // Too close to the wall - turn left
      SharpTurnLeft();
    }
    else
    {
      // Too far from the wall - turn right
      SharpTurnRight();
    }
  }

  printDebug();

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(500);
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
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void setPIDTunings()
{
  double gap = abs(setpoint-input); // Distance away from setpoint
  if (gap < 10)
  {
    // Close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    // Far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
}

void printDebug()
{
  #ifdef DEBUG
  Serial.print("sensorShort = " );
  Serial.print(sensorValueShort);
  Serial.print(" sensorLong = ");
  Serial.println(sensorValueLong);

  Serial.print("PID output: ");
  Serial.println(output);
  #endif
}
