/* Encoder Test
*** Description ***
Demonstrates how to count turns of an encoder (Lego pulley) wheel. In addition to the wheel you will need:
--A DC Motor.
--A micro switch.
--A break-beam sensor.
--An LCD screen (optional).
The wheel should be attached to a rotating axle, directly or indirectly attached to the motor, and placed
near the break-beam sensor so that as it turns it alternately blocks and allows the beam through.
*** History ***
--10/4/12:  Modified by Blane for readability and simpler logic.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>

// Define constants:
#define switchPin       5
#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).
#define leftEncoderPin  2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt)
#define rightEncoderPin 3
#define LeftMotorPin    3   // Left motor is connected to this pin
#define RightMotorPin   4   // Right motor is connected to this pin

// Define (and initialize) global variables:
volatile int encoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
int encoderCountGoal = 100;

// Define serial display and motor objects:
SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin);
AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

void setup()
{
  // Setup hardware interrupt:
  int leftInterruptPin = leftEncoderPin - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt(leftInterruptPin, IncrementAndDisplay, FALLING);   // Attach interrupt pin, name of function to be called
  // During interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  int rightInterruptPin = rightEncoderPin - 2;
  attachInterrupt(rightInterruptPin, IncrementAndDisplay, FALLING);

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
  Right_Motor.setSpeed(180);
  Left_Motor.setSpeed(180);

  mySerial.print("?f");                // Clears LCD screen
  mySerial.print("?x00?y0");           // Sets Cursor to x00,y0
  mySerial.print("Press to Begin..."); // Displays "Press to Begin..."
}

void loop()
{
  while (digitalRead(switchPin)) {
    // Wait until switch is pressed.
  }

  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Counts:");

  DriveForward();

  encoderCount = 0;
  while (encoderCount < encoderCountGoal) { }   // Wait until encoder count goal is reached
        // (allowing interrupt to update value of encoderCount during this time):

  StopMotors();
}

void IncrementAndDisplay()
{
  ++encoderCount;
  displayEncoderCounts();
}

void displayEncoderCounts()
{
  mySerial.print("?x00?y1");
  mySerial.print(encoderCount);
}

void StopMotors()
{
  Right_Motor.run(RELEASE);
  Left_Motor.run(RELEASE);
}

void DriveForward()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}
