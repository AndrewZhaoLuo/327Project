#include <Wire.h>
float V = 0.0;
int fs = 10;

// Reading value from analog input

void setup(void) 
{
  Serial.begin(9600);
}

void loop(void) 
{
  int sensorValue = analogRead(A0); 
  Serial.println(sensorValue);  
  delay(1000/fs); 
}

