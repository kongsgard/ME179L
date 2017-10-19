// Hammerhead Code
// When 'bumped' Hammerhead will move forward until an object is hit, then back up and hit it again... and again...

// Include the needed libraries
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

#define DEBUG 1  // By setting DEBUG to either 0 or 1, the program will exclude or include the print statements when it compiles

// Define constants
#define LCDTxPin        13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin      4   // Not used by LCD Board, can be any unused pin
#define StartStopButton 11   // Pin to be used by button to either start or stop the robot
#define LeftMotorPin    3   // Left motor is connected to this pin
#define RightMotorPin   4   // Right motor is connected to this pin
#define SPEED           200 // Set speed to be used for motors

enum pressedSwitch{
  NONE,
  LEFT,
  RIGHT,
  BOTH
};

static pressedSwitch hit = NONE;
static pressedSwitch previousHit = NONE;
static unsigned int backAndForthCounter = 0;

SoftwareSerial mySerial = SoftwareSerial(DummyRxPin, LCDTxPin);  // Change Tx and Rx Pins to pins of our choosing

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

#define BackwardDelay 1500 // The robot should back up for this amount of time
#define TurningDelay 1000 // The robot should turn for this amount of time

//IR sensor data input
const int analogInPinL = A0;  // Analog input from left reflector
const int analogInPinR = A1; // Analog input from right reflector

int sensorValueL = 0;        // value read on left
int sensorValueR = 0;        // value read on right

void setup()
{
  pinMode(LCDTxPin, OUTPUT);
  mySerial.begin(9600); // Set baud rate for the LCD serial communication
  mySerial.print("?f"); // Send clear screen command to LCD

  pinMode(StartStopButton, INPUT);      // Make start/stop pin an input
  digitalWrite(StartStopButton, HIGH);  // Enable the pull-up resistor on start/stop pin

  //initialize the variables we're linked to
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);

  mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
  mySerial.print("Press to begin...");

  while(digitalRead(StartStopButton))
  {
   // Wait for switch to be pressed
  }

  mySerial.print("?x00?y1");  // Move cursor to position x=0 and y=1 on the LCD display
  mySerial.print("Starting!");
  delay(1000);
  mySerial.print("?f"); // Send clear screen command to LCD
}

void loop()
{
  // read the analog on the left:
  sensorValueL = analogRead(analogInPinL);
  // read the analog value on the right:
  sensorValueR = analogRead(analogInPinR);
  // print the results to the serial monitor:
  #ifdef DEBUG
    mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("Onward!");
  #endif

  DriveForward();
  TurnRight();

  previousHit = hit;

  while (true)
  {
    if (!digitalRead(LeftSwitchPin))
    {
      hit = LEFT;
      break;
    }

    if (!digitalRead(RightSwitchPin))
    {
      hit = RIGHT;
      break;
    }

    if (!digitalRead(StartStopButton))
    {
      StopMotors();

      #ifdef DEBUG
        mySerial.print("?f"); // Send clear screen command to LCD
        mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("STOP!");
      #endif

      while(true)
      {
        delay(250);

        if(!digitalRead(StartStopButton))
        {
          DriveForward();

          #ifdef DEBUG
            mySerial.print("?f"); // Send clear screen command to LCD
            mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
            mySerial.print("Onward!");
          #endif

          delay(250);
          break;
        }
      }
    }

    delay(10);
  }

  #ifdef DEBUG
    mySerial.print("?f"); // Send clear screen command to LCD
    switch(hit)
    {
      case LEFT:
        mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("LEFT switch");
        mySerial.print("?x00?y1");  // Move cursor to position x=0 and y=1 on the LCD display
        mySerial.print("pressed!");
        break;
      case RIGHT:
        mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("RIGHT switch");
        mySerial.print("?x00?y1");  // Move cursor to position x=0 and y=1 on the LCD display
        mySerial.print("pressed!");
        break;
    }
  #endif

  if (hit == RIGHT)
  {
    DriveBackward();
    delay(BackwardDelay);
    TurnLeft();
    delay(TurningDelay);
  }

  if (hit == LEFT)
  {
    DriveBackward();
    delay(BackwardDelay);
    TurnRight();
    delay(TurningDelay);
  }

  if (hit != previousHit)
  {
    backAndForthCounter++;
    if (backAndForthCounter > 2)
    {
      #ifdef DEBUG
        mySerial.print("?f"); // Send clear screen command to LCD
        mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("Stuck!");
      #endif

      TurnRight();
      delay(2*TurningDelay);

      backAndForthCounter = 0;
    }
  }
  else
  {
    backAndForthCounter = 0;
  }

  #ifdef DEBUG
    mySerial.print("?f"); // Send clear screen command to LCD
    mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("Onward!");
  #endif
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
  delay(1000);
}

void DriveBackward()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void TurnLeft()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
  delay(3000);
}

void TurnRight()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
  delay(3000);
}
