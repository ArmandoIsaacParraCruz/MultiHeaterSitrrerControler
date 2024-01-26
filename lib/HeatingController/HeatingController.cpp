#include "HeatingController.h"


const double HeatingController::MAX_NUMBER_OF_SEMICYCLES = 120;
const double HeatingController::MIN_NUMBER_OF_SEMICYCLES = 0;
uint8_t HeatingController::csHeatingManager;
const char  HeatingController::startCommunicationFlag = 255; 


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


void HeatingController::setCsHeatingManagerPin(uint8_t _csHeatingManager)
{
    csHeatingManager = _csHeatingManager;
    pinMode(csHeatingManager, OUTPUT);
    digitalWrite(csHeatingManager, HIGH);

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

void HeatingController::transferSemicycles(uint8_t semicycles[NUMBER_OF_PLACES])
{
    digitalWrite(csHeatingManager, LOW);
    SPI.transfer(startCommunicationFlag);
    delayMicroseconds(50);
    digitalWrite(csHeatingManager, HIGH);
    for(uint8_t i = 0; i < NUMBER_OF_PLACES; ++i) {
        Serial.print(semicycles[i]);
        Serial.print(" ");
        digitalWrite(csHeatingManager, LOW);
        SPI.transfer(semicycles[i]);
        delayMicroseconds(50);
        digitalWrite(csHeatingManager, HIGH);
    }
}
