#include <AFMotor.h>
#include <SoftwareSerial.h>

// Define constants
#define LCDTxPin        13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin      4   // Not used by LCD Board, can be any unused pin
#define LeftMotorPin    3   // Left motor is connected to this pin
#define RightMotorPin   4   // Right motor is connected to this pin
#define SPEED           250 // Set speed to be used for motors

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

void setup()
{
  Right_Motor.setSpeed(0.25 * SPEED);
  Left_Motor.setSpeed(SPEED);
}

void loop()
{
  DriveForward();
}

void DriveForward()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}
