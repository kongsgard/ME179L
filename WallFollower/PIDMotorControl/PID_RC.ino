// Include encoder and PID libraries
#include <AFMotor.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

// Define pins
#define LeftMotorPin  3
#define RightMotorPin  4
#define LCDTxPin     13
#define rxPin          6


const int ShortIR = A0;
const int LongIR = A1;

int sensorValueShort = 0;     // Short range sensor value
int sensorValueLong = 0;

// Define PID parameters
double Setpoint, Input, Output;
double Kp = 0.7;
double Ki = 0.05;
double Kd = 0.15;
PID PID_1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

AF_DCMotor Left_Motor(LeftMotorPin, MOTOR34_1KHZ); // Set up left motor on port 4, 1KHz pwm
AF_DCMotor Right_Motor(RightMotorPin, MOTOR34_1KHZ); // Set up right motor on port 3, 1KHz pwm
SoftwareSerial mySerial = SoftwareSerial(rxPin, LCDTxPin);

void setup()
{
  // Set pin modes
  pinMode(LeftMotorPin, OUTPUT);
  pinMode(RightMotorPin,OUTPUT);
  pinMode(ShortIR, INPUT);

  // Set initial positions
  Setpoint = 300;

  // Set PID limits and mode
  PID_1.SetMode(AUTOMATIC);

  // Open serial port
  Serial.begin(9600);
}


void loop()
{
  // Read transition switch and rudder stick
  sensorValueShort = analogRead(ShortIR)
  diff = sensorValueShort - Setpoint;
  Input = abs(sensorValueShort - Setpoint);

  // Run PID controller
  PID_1.Compute();

  Serial.print("Output: ");
  Serial.println(Output);

  // Apply proper motor direction to output
  if (diff > 0 && diff <= 50){
    Right_Motor.setSpeed(Output);
    Left_Motor.setSpeed(0.6*Output);
  }
  else if (diff < 0 && diff >= -50){
    Right_Motor.setSpeed(0.6*Output);
    Left_Motor.setSpeed(Output);
  }
  else if (diff > 50){
    Right_Motor.setSpeed(Output);
    Left_Motor.setSpeed(0.3*Output);
  }
  else if (diff < -50){
    Right_Motor.setSpeed(0.3*Output);
    Left_Motor.setSpeed(Output);
  }
  else {
    Right_Motor.setSpeed(Output);
    Left_Motor.setSpeed(Output);
  }

}
