// Include encoder and PID libraries
#include <Encoder.h>
#include <PID_v1.h>

// Define pins
Encoder Enc_1(6, 16);
int Motor_1 = 9;
int Dir_1 = 13;
int Trans_Pin = 12;
int Rudder_Pin = 14;

// Initialize PWM inputs
int Trans_PWM;
int Rudder_PWM;

// Define PID parameters
double Setpoint, Input, Output;
double Kp = 0.7;
double Ki = 0.01;
double Kd = 0.015;
PID PID_1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  // Set pin modes
  pinMode(Motor_1, OUTPUT);
  pinMode(Dir_1, OUTPUT);
  pinMode(Trans_Pin, INPUT);

  // Set initial positions
  Input = Enc_1.read();
  Setpoint = 0.5837*pulseIn(Trans_Pin, HIGH) + 1095;

  // Set PID limits and mode
  PID_1.SetOutputLimits(-255,255);
  PID_1.SetMode(AUTOMATIC);

  // Open serial port
  Serial.begin(9600);
}

// Store last inputs
long oldPosition = Input;

void loop()
{
  // Read transition switch and rudder stick
  Trans_PWM = pulseIn(Trans_Pin, HIGH);
  Rudder_PWM = pulseIn(Rudder_Pin, HIGH);

  // Mix and compute desired motor position
  Setpoint = (72963*pulseIn(Trans_Pin, HIGH)/125000 + 1095)+(Rudder_PWM-1500)*0.6;

  // Read encoder
  Input = Enc_1.read();

  // Update serial monitor with encoder information
  if (Input != oldPosition)
  {
    Serial.println(Setpoint-Input);
    oldPosition = Input;
  }

  // Run PID controller
  PID_1.Compute();

  // Apply proper motor direction to output
  if (Output >= 0 && Output <= 255)
  {
    analogWrite(Motor_1, Output);
    digitalWrite(Dir_1, 0);
  }
  else if (Output >= -255 && Output < 0)
  {
    analogWrite(Motor_1, -1*Output);
    digitalWrite(Dir_1, 1);
  }
