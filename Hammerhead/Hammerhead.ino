// Hammerhead Code
// When 'bumped' Hammerhead will move forward until an object is hit, then back up and hit it again... and again...

// Include the needed libraries
#include <AFMotor.h>
#include <SoftwareSerial.h>

#define DEBUG 1  // By setting DEBUG to either 0 or 1, the program will exclude or include the print statements when it compiles

// Define constants
#define LeftSwitchPin  14  // Left switch is connected to this pin
#define RightSwitchPin 15  // Right switch is connected to this pin
#define LCDTxPin       13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin     4   // Not used by LCD Board, can be any unused pin  
#define LeftMotorPin   3   // Left motor is connected to this pin
#define RightMotorPin  4   // Right motor is connected to this pin
#define SPEED          200 // Set speed to be used for motors

enum pressedSwitch{
  NONE,
  LEFT,
  RIGHT,
  BOTH
};

static pressedSwitch lastHit = NONE;

SoftwareSerial mySerial = SoftwareSerial(DummyRxPin, LCDTxPin);  // Change Tx and Rx Pins to pins of our choosing

AF_DCMotor Left_Motor(3, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(4, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm

void setup()
{
  pinMode(LCDTxPin, OUTPUT);
  mySerial.begin(9600); // Set baud rate for the LCD serial communication
  mySerial.print("?f"); // Send clear screen command to LCD
  
  pinMode(LeftSwitchPin, INPUT);      // Make left switch pin an input
  digitalWrite(LeftSwitchPin, HIGH);  // Enable the pull-up resistor on left switch pin
  pinMode(RightSwitchPin, INPUT);     // Make right switch pin an input
  digitalWrite(RightSwitchPin, HIGH); // Enable the pull-up resistor on right switch pin
  
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);
  
  mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
  mySerial.print("Bump to begin...");
  
  while(digitalRead(LeftSwitchPin))
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
  #ifdef DEBUG
    mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("Onward!");
  #endif

  DriveForward();

  while (true)
  {
    if (!digitalRead(LeftSwitchPin))
    {
      lastHit = LEFT;
      break;
    }
  
    if (!digitalRead(RightSwitchPin))
    {
      lastHit = RIGHT;
      break;
    }
    
    delay(100);
  }

  if (lastHit == RIGHT)
  {
    DriveBackward();
    delay(2000);
    TurnLeft();
    delay(2000);
  }
  
  if (lastHit == LEFT)
  {
    DriveBackward();
    delay(2000);
    TurnRight();
    delay(2000); 
  }

  #ifdef DEBUG
    if (lastHit == BOTH)
    {
      mySerial.print("?x00?y0");  // Move cursor to position x=0 and y=0 on the LCD display
      mySerial.print("Both switches pressed!");
    }
  #endif
}

void DriveForward()
{
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
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
}

void TurnRight()
{
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

