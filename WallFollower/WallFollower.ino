#include <PID_v1.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>

// Pin constants:
#define switchPin       11
#define leftSwitchPin   14  // Left switch is connected to this pin
#define rightSwitchPin  15  // Right switch is connected to this pin
#define potPin          2
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define leftEncoderPin  2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt)
#define rightEncoderPin 3
#define LeftMotorPin    5   // Left motor is connected to this pin
#define RightMotorPin   6   // Right motor is connected to this pin

// Define (and initialize) global variables:
volatile int leftEncoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int rightEncoderCount;
int encoderCountGoal = 10;

const int analogInPinL = A0;  // Analog input from left reflector
const int analogInPinR = A1; // Analog input from right reflector

int sensorValueL = 0;        // value read on left
int sensorValueR = 0;        // value read on right

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 3, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 4, 1KHz pwm

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup() {
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

  // Setup switch:
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);

  // Set motor speed:
  Right_Motor.setSpeed(speedSettings[1]);
  Left_Motor.setSpeed(speedSettings[1]);

  while (digitalRead(switchPin))
  {
    // Wait until switch is pressed.
    // Change settings here
    delay(150);
  }
}

void loop() {
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
  delay(100);
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
