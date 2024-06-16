#pragma once
#include <Arduino.h>
#include <Wire.h>

class MCP23017
{
    private:
        static int _address;
        
    public:
        MCP23017();
        static void begin(uint8_t SDA, uint8_t SCL, int address);
        static void digitalWrite(byte pin, bool value);
};
