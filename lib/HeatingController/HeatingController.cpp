#include "HeatingController.h"


const double HeatingController::MAX_NUMBER_OF_SEMICYCLES = 120;
const double HeatingController::MIN_NUMBER_OF_SEMICYCLES = 0;


HeatingController::HeatingController(   double _kp,
                                        double _ki,
                                        double _kd,
                                        uint8_t _csPin)
    : Controller(_kp, _ki, _kd)
{
    csPin = _csPin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}

void HeatingController::adjustOutputSignalManually(uint8_t _semicycles)
{
    
}

void HeatingController::adjustOutputSignal()
{
    
}


void HeatingController::updateInput()
{
    input = getTemperature();
    computeOutput();
}



float HeatingController::getTemperature()
{
    digitalWrite(csPin, LOW); // Activar sensor
    uint16_t v;

    // Leer datos
    byte msb = SPI.transfer(0x00);
    byte lsb = SPI.transfer(0x00);

    digitalWrite(csPin, HIGH); // Desactivar sensor

    // Convertir bits a temperatura
    int temperature = ((msb << 8) | lsb) >> 3;

    float celsius = temperature * 0.25;
    return celsius;
}
