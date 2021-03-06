#include <AFMotor.h>
#include <SoftwareSerial.h>

// Define constants
#define LCDTxPin        13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin      4   // Not used by LCD Board, can be any unused pin
#define SPEED           200 // Set speed to be used for motors

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

void setup()
{
  Left_Motor.setSpeed(SPEED);
  Right_Motor.setSpeed(SPEED);
}

void loop()
{
  DriveForward();
}

void DriveForward()
{
  Left_Motor.run(BACKWARD);
  Right_Motor.run(FORWARD);
}
