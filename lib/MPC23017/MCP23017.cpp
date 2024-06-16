#include "MCP23017.h"

int MCP23017::_address;

MCP23017::MCP23017()
{
   
}

void MCP23017::begin(uint8_t SDA, uint8_t SCL, int address)
{
    _address = address;
    Wire.begin(SDA, SCL);
     // Configurar los registros de configuración del MCP23017
    // Configurar todos los pines como salida (IODIRA y IODIRB)
    Wire.beginTransmission(_address);
    Wire.write(0x00); // Registro IODIRA (pines 0-7)
    Wire.write(0x00); // Configurar todos los pines de Port A como salida
    Wire.write(0x00); // Configurar todos los pines de Port B como salida
    Wire.endTransmission();
}

void MCP23017::digitalWrite(byte pin, bool value)
{
    byte port, bit;
  if (pin < 8) {
    port = 0; // Port A
    bit = pin;
  } else {
    port = 1; // Port B
    bit = pin - 8;
  }
  
  // Leer el estado actual del puerto
  Wire.beginTransmission(_address);
  Wire.write(port == 0 ? 0x12 : 0x13); // Dirección del registro GPIOA o GPIOB
  Wire.endTransmission(false);
  Wire.requestFrom(_address, 1);
  byte currentValue = Wire.read();
  
  // Modificar el bit correspondiente
  bitWrite(currentValue, bit, value);
  
  // Escribir el nuevo valor en el puerto
  Wire.beginTransmission(_address);
  Wire.write(port == 0 ? 0x12 : 0x13); // Dirección del registro GPIOA o GPIOB
  Wire.write(currentValue);
  Wire.endTransmission();
}
