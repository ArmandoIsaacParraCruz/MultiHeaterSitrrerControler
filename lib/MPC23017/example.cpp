/*
#include <Arduino.h>
#include <Wire.h>
#include "MCP23017.h"
#include "StirringController.h"

#define MCP23017_ADDRESS 0x20 // Dirección del MCP23017 (A0-A2 conectados a GND)
#define SDA 8
#define SCL 18

MCP23017 mcp;

void setup() {
  mcp.begin(SDA, SCL, MCP23017_ADDRESS);
}

void loop() {
  mcp.digitalWrite(12, LOW);
  delay(1000);
  mcp.digitalWrite(12, HIGH);
  delay(1000);
}

*/