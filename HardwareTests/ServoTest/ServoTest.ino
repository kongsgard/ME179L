#include <Servo.h>

#define servo1Pin 9
#define servo2Pin 10

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo

void setup() {
  servo1.attach(servo1Pin);  // attaches the servo on pin 9 to the servo object
  servo2.attach(servo2Pin);  // attaches the servo on pin 10 to the servo object
}

void loop() {
  servo1.write(52); // Move to 52 degrees
  servo2.write(135); // Move to 135 degrees
}
