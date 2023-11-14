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
    : Controller(_kp, _ki, _kd), max6675Sensor(_clkPin, _csPin, _soPin)
{
    heatingResistorPin = _heatingResistorPin;
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
    input = double(max6675Sensor.readCelsius());
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