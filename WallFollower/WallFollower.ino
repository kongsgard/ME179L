/*
two Analog inputs and serial output
*/


// Pin constants:
#define switchPin       11

const int analogInPinL = A0;  // Analog input from left reflector
const int analogInPinR = A1; // Analog input from right reflector

int sensorValueL = 0;        // value read on left
int sensorValueR = 0;        // value read on right


void setup() {
  // Setup switch:
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  /*
  while (digitalRead(switchPin))
  {
    // Wait until switch is pressed.
    // Change settings here
    delay(150);
  }
  */

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
