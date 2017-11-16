// RoboRat - a cheese picking robot

#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

// Pin defines:
#define killSwitchPin  2
#define leftMotorPin   3
#define RightMotorPin  4
#define armServoPin    9
#define towerServoPin  10
#define lightSensorPin A2

#define SPEED 255
AF_DCMotor Left_Motor(leftMotorPin, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm

// TODO: define servo angle constants on "attack" mode
Servo armServo;  // create servo object to control a servo
Servo towerServo;  // create servo object to control a servo

const unsigned int lightSensorThreshold = 200;

void setup() {
  armServo.attach(armServoPin);
  towerServo.attach(towerServoPin);

  pinMode(killSwitchPin, INPUT);
  digitalWrite(killSwitchPin, HIGH);

  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  armServo.write(180); // Move to 0 degrees
  towerServo.write(180); // Move to 120 degrees

  while(digitalRead(killSwitchPin) && analogRead(lightSensorPin) > lightSensorThreshold) {}
}

void loop() {
  armServo.write(140); // Move to 0 degrees
  towerServo.write(105); // Move to 120 degrees

  DriveForward();
  delay(1000);

//  TurnLeft();
//  delay(200);
//
//  DriveForward();
//  delay(1000);
//
//  TurnRight();
//  delay(200);
//
//  DriveForward();
//  delay(5000);
//
}


void DriveForward(){
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void DriveBack(){
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void TurnLeft() {
  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
}

void TurnRight() {
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void StopMotors(){
  Right_Motor.run(RELEASE);
  Left_Motor.run(RELEASE);
}
