#include <Wire.h>
#include <Adafruit_ADS1015.h>

// ADS1115 uses I2C, so devices have addresses
// Wired the ADS1115 to have address 0x48
Adafruit_ADS1115 ads(0x48);

int V = 0;
int fs = 200;

int16_t offset = 0;

void setup(void) 
{
  Serial.begin(9600);
  
  // Why this is so conveniant
  ads.setGain(GAIN_EIGHT);
  ads.begin();
 
  // consider this your DC offset 
  offset = ads.readADC_SingleEnded(0);
}

void loop(void) 
{
  int16_t adc0;

  adc0 = ads.readADC_SingleEnded(0);
  V = adc0 - offset; 
  
  // print out raw num from [0, 2^16)
  Serial.println(V);  
  delay(1000/fs); 
}

