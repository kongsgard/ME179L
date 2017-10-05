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
#define switchPin 3   
#define txPin 13   // LCD tx pin.
#define rxPin 13   // LCD rx pin (not really used).
#define encoderPin 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define motorTerminal 4   // (1-4 only).

// Define (and initialize) global variables:
volatile int encoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
int encoderCountGoal = 10;

// Define serial display and motor objects:
SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin);  
AF_DCMotor myMotor( motorTerminal, MOTOR34_1KHZ); 


void setup() {
  
  // Setup hardware interrupt:
  int interruptPin = encoderPin - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPin, IncrementAndDisplay, FALLING);   // Attach interrupt pin, name of function to be called 
        // during interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  
  // Setup encoder i.e. break-beam:
  pinMode( encoderPin, INPUT); 
  digitalWrite( encoderPin, HIGH);  
  
  // Setup switch:
  pinMode( switchPin, INPUT); 
  digitalWrite( switchPin, HIGH);  
  
  // Setup serial display:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 
        
  // Set motor speed:
  myMotor.setSpeed(180);
  
  mySerial.print("?f");          //Clears LCD screen
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
  mySerial.print("Press to Begin...");    //Displays "Press to Begin..."
}


void loop() {

  while (digitalRead(switchPin)) { }         // Wait until switch is pressed.
  
  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Counts:");
  
  
  myMotor.run(FORWARD);
  myMotor.run(FORWARD);
  
  encoderCount = 0;  
  while (encoderCount < encoderCountGoal) { }   // Wait until encoder count goal is reached 
        // (allowing interrupt to update value of encoderCount during this time):
  
  myMotor.run(RELEASE); 
}


void IncrementAndDisplay() {
  ++encoderCount;
  mySerial.print("?x00?y1");
  mySerial.print(encoderCount);  
}

