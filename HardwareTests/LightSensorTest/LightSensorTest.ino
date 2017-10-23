#include <SoftwareSerial.h>

const int LightSensorPin = A3;

void setup(){
  pinMode(LightSensorPin, INPUT);
  digitalWrite(LightSensorPin, HIGH);

  Serial.begin(9600);
}
void loop(){
  Serial.print("Light Sensor Value:");
  Serial.println(analogRead(LightSensorPin));
  delay(100);
}
