#include "HeatingController.h"

volatile uint32_t HeatingController::lastZeroCrossingTime;
volatile uint32_t HeatingController::semicyclesCounter;
uint32_t HeatingController::maxNumberOfSemicycles;
uint8_t HeatingController::zeroCrossingPin;


HeatingController::HeatingController(   uint8_t _heatingResistorPin, 
                                        double _kp,
                                        double _ki,
                                        double _kd,
                                        uint8_t _clkPin,
                                        uint8_t _csPin, 
                                        uint8_t _soPin)
    : Controller(_kp, _ki, _kd)
{
    heatingResistorPin = _heatingResistorPin;
    clkPin = _clkPin;
    csPin = _csPin; 
    soPin = _soPin; 
    lastZeroCrossingTime = micros();
    semicyclesCounter = 0;
    maxNumberOfSemicycles = 120;
}



void HeatingController::adjustOutputSignal()
{
    if(HeatingController::semicyclesCounter <= output){
        digitalWrite(heatingResistorPin, HIGH);
    } else {
        digitalWrite(heatingResistorPin, LOW);
    }
}

void HeatingController::updateInput()
{
    input = getTemperature();
    computeOutput();
}



void HeatingController::configureHeatingControllerPins(uint8_t &_zeroCrossingPin, void (*_zeroCrossingInterruptFunction)())
{
    pinMode(heatingResistorPin, OUTPUT);
    zeroCrossingPin = _zeroCrossingPin;
    pinMode(_zeroCrossingPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_zeroCrossingPin), _zeroCrossingInterruptFunction, FALLING);
}

HeatingController::~HeatingController()
{
    digitalWrite(heatingResistorPin, LOW);
    pinMode(heatingResistorPin, INPUT);
    ledcDetachPin(zeroCrossingPin);
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
