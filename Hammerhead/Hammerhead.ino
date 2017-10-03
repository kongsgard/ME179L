// Hammerhead Code
// When 'bumped' Hammerhead will move forward until an object is hit, then back up and hit it again... and again...

// Include the needed libraries
#include <AFMotor.h>
#include <SoftwareSerial.h>

#define DEBUG 1  // By setting DEBUG to either 0 or 1, the program will exclude or include the print statements when it compiles

// Define Constants
#define SwitchPin  14  // Switch is connected to this pin
#define LCDTxPin   13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin 4   // Not used by LCD Board, can be any unused pin  
#define SPEED      200 // Set speed to be used for motors

SoftwareSerial mySerial =  SoftwareSerial(DummyRxPin, LCDTxPin);  //Change Tx and Rx Pins to pins of our choosing
AF_DCMotor Right_Motor(3, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm
AF_DCMotor Left_Motor(4, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm

void setup(){
  pinMode(LCDTxPin, OUTPUT);
  mySerial.begin(9600);  //Set baud rate for the LCD serial communication
  mySerial.print("?f"); //Sends clear screen command to LCD
  pinMode(SwitchPin, INPUT); //Makes Switch Pin an input
  digitalWrite(SwitchPin, HIGH);   //Enables the pull-up resistor on Switch Pin
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);
  
  mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
  mySerial.print("Bump to begin...");
  while(digitalRead(SwitchPin)){
   //Wait for switch to be pressed 
  }
  mySerial.print("?x00?y1");  //Move cursor to position x=0 and y=1 on the LCD display
  mySerial.print("Starting!");
  delay(1000);
  mySerial.print("?f"); //Sends clear screen command to LCD
  
}

void loop(){
  
  #ifdef DEBUG
    mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("Onward!");
  #endif
  
  DriveForward();

  while(digitalRead(SwitchPin)){   
    //Wait for switch to be pressed 
  }
  
  #ifdef DEBUG
    mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("OUCH!!!");
  #endif
  
  DriveBackward();
  delay(2000);
    
}

void DriveForward(){
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void DriveBackward(){
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

  

