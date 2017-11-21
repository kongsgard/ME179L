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
#define settingsPin    18 // A4
#define frontRangePin  A4
#define sideRangePin   A5

unsigned int SPEED = 255;
AF_DCMotor Left_Motor(leftMotorPin, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm

#define black 300

// TODO: define servo angle constants on "attack" mode
Servo armServo;  // create servo object to control a servo
Servo towerServo;  // create servo object to control a servo

const unsigned int lightSensorThreshold = 200;

unsigned int leftIRValue      = 0;
unsigned int rightIRValue     = 0;
unsigned int sideSensorValue  = 0;
unsigned int frontSensorValue = 0;

unsigned int lane = 0;

unsigned long time;

// P-controller
float gain1;
int   error1;
int   offset1;
float gain2;
int   error2;
int   offset2;

int leftMotorSpeed;
int rightMotorSpeed;

// --- //

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
    //changeLaneSetting();
    lane = 1; // TODO: remove
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
      followWall();
      break;
    case 2:
      followMiddleLane();
      // when close to wall, change lane to 3
      break;
    case 3:
      followWall();
      break;
  }

  killSwitch();

  #ifdef DEBUG
  printDebug();
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
}

void followWall()
{
  sideSensorValue = analogRead(sideRangePin);
  frontSensorValue = analogRead(frontRangePin);

  gain1 = -1.5;
  error1 = 125 - sideSensorValue;
  offset1 = gain1 * error1;

  leftMotorSpeed = min(255, max(0, SPEED + offset1));
  rightMotorSpeed = min(255, max(0, SPEED - 2*offset1));

  Left_Motor.setSpeed(leftMotorSpeed);
  Right_Motor.setSpeed(rightMotorSpeed);

  gain2 = -10;
  error2 = 200 - frontSensorValue;
  offset2 = gain2 * error2;

  if (error2 < 0)
  {
    Left_Motor.setSpeed(min(255, max(0, SPEED + offset2)));
    Right_Motor.setSpeed(min(255, max(0, SPEED - offset2)));
  }

  Serial.print("offset2: ");
  Serial.println(offset2);

  Serial.print("leftMotorSpeed: ");
  Serial.println(leftMotorSpeed);
  Serial.print("rightMotorSpeed: ");
  Serial.println(rightMotorSpeed);

  DriveForward();
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

  Serial.print("sideRangeValue = ");
  Serial.println(analogRead(sideRangePin));

  Serial.print("frontRangeValue = ");
  Serial.println(analogRead(frontRangePin));

  delay(500);
}
