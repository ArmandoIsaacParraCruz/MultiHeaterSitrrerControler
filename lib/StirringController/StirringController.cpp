#include "StirringController.h"

StirringController::StirringController( uint8_t _motorAPin, 
                                        uint8_t _encoderPhaseAPin,
                                        uint8_t _channel,
                                        float _pulsesPerRevolution,
                                        double _kp,
                                        double _ki,
                                        double _kd)
    : Controller(_kp, _ki, _kd)
{
    motorAPin = _motorAPin;
    encoderPhaseAPin = _encoderPhaseAPin;
    channel = _channel;
    pulsesPerRevolution = _pulsesPerRevolution;
    pulses = 0;
    debouncePreviousTime = micros();
    lastMeasurementTime = millis();
}



void StirringController::adjustOutputSignal()
{
    noInterrupts();
    updateInput();
    computeOutput();
    ledcWrite(channel, output);
    interrupts();
}

void StirringController::updateInput()
{
    
    double measure = double((60.0 * 1000.0 / pulsesPerRevolution) / (millis() - lastMeasurementTime) * pulses) / 2.0;
    input = alpha * measure + (1.0 - alpha) * input;
    pulses = 0;
    lastMeasurementTime= millis();
    
}


void StirringController::configureStirringControllerPins(   uint32_t &_frequency, 
                                                            uint8_t &_resolution,
                                                            void (*interruptFunction)())
{
    pinMode(encoderPhaseAPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPhaseAPin), interruptFunction, RISING);
    pinMode(motorAPin, OUTPUT);
    ledcSetup(channel, _frequency, _resolution);
    ledcAttachPin(motorAPin, channel);
}


void StirringController::incrementPulses()
{
    if(micros() - debouncePreviousTime > DEBOUNCE_TIME){
        ++pulses;
        debouncePreviousTime = micros();
    }  
}

StirringController::~StirringController()
{
    digitalWrite(motorAPin, LOW);  
    pinMode(motorAPin, INPUT);      
    ledcDetachPin(motorAPin);
    pinMode(encoderPhaseAPin, INPUT);       
    detachInterrupt(digitalPinToInterrupt(encoderPhaseAPin));
}


