#include <SoftwareSerial.h>

#define txPin           13  // LCD tx pin.
#define rxPin           13  // LCD rx pin (not really used).

const int LightSensorPin = A3;

// LCD Screen:
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup(){
  pinMode(LightSensorPin, INPUT);
  digitalWrite(LightSensorPin, HIGH);

  // Setup serial display:
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);

  // Clear LCD screen
  mySerial.print("?f"); // Send clear screen command to LCD

  Serial.begin(9600);
}

void loop(){
  //Serial.print("Light Sensor Value:");
  Serial.println(analogRead(LightSensorPin));

  mySerial.print("?x00?y1");
  mySerial.print("L: ");
  mySerial.print(analogRead(LightSensorPin));

  delay(100);
}
