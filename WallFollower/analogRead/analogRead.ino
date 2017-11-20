/*
one Analog inputs and serial output
*/

// These constants won't change.  They're used to give names
// to the pins used:

const int analogInPin = A5;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog
  int sensorValue = analogRead(analogInPin);

  Serial.print("sensor = " );
  Serial.println(sensorValue);

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);
}
