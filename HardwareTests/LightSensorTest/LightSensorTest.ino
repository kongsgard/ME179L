#include <SoftwareSerial.h>

#define LCDTxPin        13  // LCD connected to this pin (14 is analog 0)
#define DummyRxPin      4   // Not used by LCD Board, can be any unused pin

const int LightSensorPin = A3;
int lsv;
SoftwareSerial mySerial = SoftwareSerial(DummyRxPin, LCDTxPin);  // Change Tx and Rx Pins to pins of our choosing

void setup(){
  pinMode(LCDTxPin, OUTPUT);
  pinMode(LightSensorPin, INPUT);
  digitalWrite(LightSensorPin, HIGH);

  lsv = analogRead(LightSensorPin);
  //Serial.begin(9600);
  mySerial.begin(9600);
  mySerial.print("?f");
  mySerial.print("?l");
  mySerial.print("?x00?y0");
  mySerial.print("Light Sensor Value");
}
void loop(){
mySerial.print("?f");
mySerial.print("?l");
mySerial.print("?x00?y0");
mySerial.print("Light Sensor Value");
  /*Serial.println(analogRead(LightSensorPin));
  mySerial.print("?x00?y1");
  mySerial.print(lsv);*/


  delay(100);
}
