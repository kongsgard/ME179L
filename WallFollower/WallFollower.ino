#include <PID_v1.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

// Pin constants:
#define switchPin       11
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define LeftMotorPin    3   // Left motor is connected to this pin
#define RightMotorPin   4   // Right motor is connected to this pin
#define SPEED           250 // Set speed to be used for motors

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

// Define (and initialize) global variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int encoderCountGoal = 10;

const int analogInPinL = A0;  // Analog input from left reflector
const int analogInPinR = A1; // Analog input from right reflector

int sensorValueL = 0;        // value read on left
int sensorValueR = 0;        // value read on right

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);

  // Set motor speed:
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
    DriveForward();
  // read the analog on the left:
  sensorValueL = analogRead(analogInPinL);
  // read the analog value on the right:
  sensorValueR = analogRead(analogInPinR);
  // print the results to the serial monitor:

  Serial.print("sensorL = " );
  Serial.print(sensorValueL);
  Serial.print(" sensorR = ");
  Serial.println(sensorValueR);

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

void DriveForward()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}
