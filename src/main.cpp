#include <Arduino.h>
#include <MCP3XXX.h>
#include "max6675.h"

MCP3008 adc;

void setup()
{
  Serial.begin(9600);
  adc.begin(10, 8, 7, 6);
}

void loop()
{
  Serial.println(adc.analogRead(0));
  delay(500);
}