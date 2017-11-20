// RoboRat - a cheese picking robot

#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

#define DEBUG 1

// Pin defines:
#define killSwitchPin  2
#define leftMotorPin   3
#define RightMotorPin  4
#define armServoPin    9
#define towerServoPin  10
#define txPin          13  // LCD tx pin.
#define rxPin          13  // LCD rx pin (not really used)
#define leftIRPin      A0
#define rightIRPin     A1
#define lightSensorPin A2
#define longRangePin   A3
#define settingsPin    18

#define SPEED 255
AF_DCMotor Left_Motor(leftMotorPin, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm

#define black 300

// TODO: define servo angle constants on "attack" mode
Servo armServo;  // create servo object to control a servo
Servo towerServo;  // create servo object to control a servo

const unsigned int lightSensorThreshold = 200;

unsigned int leftIRValue    = 0;
unsigned int rightIRValue   = 0;
unsigned int longRangeValue = 0;

unsigned int lane = 0;

unsigned long time;

void setup()
{
  #ifdef DEBUG
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);
  #endif

  // Motors:
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  // Servos
  armServo.attach(armServoPin);
  towerServo.attach(towerServoPin);

  armServo.write(180); // Move to 0 degrees
  towerServo.write(180); // Move to 120 degrees

  // Kill switch:
  pinMode(killSwitchPin, INPUT);
  digitalWrite(killSwitchPin, HIGH);

  // Light sensor:
  pinMode(lightSensorPin, INPUT);
  digitalWrite(lightSensorPin, HIGH);

  // IR sensors:
  pinMode(leftIRPin, INPUT_PULLUP);
  pinMode(rightIRPin, INPUT_PULLUP);

  while(digitalRead(killSwitchPin) && analogRead(lightSensorPin) > lightSensorThreshold) {
    changeLaneSetting();
  }
  armServo.write(140); // Move to 0 degrees
  towerServo.write(105); // Move to 120 degrees

  DriveForward();
  time = millis();
}

void loop()
{
  switch(lane) // Choose which lane to go for first (the GOLDEN lane!)
  {
    case 1:
      break;
    case 2:
      followMiddleLane();
    case 3;
      break:
  }

  #ifdef DEBUG
  //printDebugHost();
  //printDebugLCD();
  #endif
}

// --- //

void followMiddleLane()
{
  leftIRValue = analogRead(leftIRPin);
  rightIRValue = analogRead(rightIRPin);

  if(leftIRValue < black && rightIRValue < black)
  {
    //BOTH WHITE
    Right_Motor.setSpeed(SPEED);
    Left_Motor.setSpeed(SPEED);
  }
  else if(leftIRValue < black && rightIRValue > black)
  {
    //TURN RIGHT
    Right_Motor.setSpeed(SPEED);
    Left_Motor.setSpeed(0);
  }
  else if(leftIRValue > black && rightIRValue < black)
  {
    //TURN LEFT
    Right_Motor.setSpeed(0);
    Left_Motor.setSpeed(SPEED);
  }
  else
  {
    //BOTH BLACK
    Right_Motor.setSpeed(SPEED);
    Left_Motor.setSpeed(SPEED);
    DriveForward();
  }

  killSwitch();
}

// --- //

void DriveForward()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void DriveBack()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void TurnLeft()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
}

void TurnRight()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void StopMotors()
{
  Right_Motor.run(RELEASE);
  Left_Motor.run(RELEASE);
}

void changeLaneSetting()
{
    lane = analogRead(settingsPin) / 300 + 1;
    if(lane = 4)
    {
      lane = 3;
    }
    //Serial.print("lane = ");
    //Serial.println(lane);
}

void killSwitch()
{
  // Stop robot if killSwitch is pressed
  if (millis() - time > 1000 && !digitalRead(killSwitchPin))
  {
      StopMotors();
      while(true) {}
    }
}

void printDebug()
{
  //Serial.print("lightSensor = ");
  //Serial.println(analogRead(lightSensorPin));

  //Serial.print("leftIRValue = ");
  //Serial.println(analogRead(leftIRPin));
  //Serial.print("rightIRValue = ");
  //Serial.println(analogRead(rightIRPin));

  Serial.print("longRangeValue = ");
  Serial.println(analogRead(longRangePin));

  delay(500);
}
