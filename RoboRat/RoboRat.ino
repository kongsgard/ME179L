#include <Servo.h> 
#include <AFMotor.h>
#include <SoftwareSerial.h>

// Set up motor contacts and servo pins______________________________
#define switchPin1 2
Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo

#define SPEED 175


AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm
AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm

void setup() {
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);  // attaches the servo on pin 10 to the servo object 

  pinMode(switchPin1, INPUT); 
  digitalWrite(switchPin1, HIGH);  
  
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);


  while(digitalRead(switchPin1)) {}
}


void loop() {
  servo1.write(52); // Move to 0 degrees
  servo2.write(135); // Move to 120 degrees

  DriveForward();
}


void DriveForward(){
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void DriveBack(){
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
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
