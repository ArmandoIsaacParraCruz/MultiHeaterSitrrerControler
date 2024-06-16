#include "StirringController.h"

StirringController::StirringController(double _kp, double _ki, double _kd)
    : Controller(_kp, _ki, _kd)
{
    
}


void StirringController::begin( uint8_t _motorAPin, 
                                uint8_t _encoderPhaseAPin,
                                uint8_t _channel,
                                float _pulsesPerRevolution,
                                uint32_t _frequency, 
                                uint8_t _resolution, 
                                void (*interruptFunction)())
{
    motorAPin = _motorAPin;
    encoderPhaseAPin = _encoderPhaseAPin;
    channel = _channel;
    pulsesPerRevolution = _pulsesPerRevolution;
    pulses = 0;
    pinMode(motorAPin, OUTPUT);
    pinMode(encoderPhaseAPin, INPUT_PULLUP);
    attachInterrupt(encoderPhaseAPin, interruptFunction, FALLING);
    analogWriteResolution(_resolution); 
    analogWriteFrequency(_frequency);
    debouncePreviousTime = micros();
    lastMeasurementTime = millis();
}




void StirringController::adjustOutputSignal()
{
    computeOutput();
    analogWrite(motorAPin, output);;
}

void StirringController::updateInput()
{
    double measure = double((60.0 * 1000.0 / pulsesPerRevolution) / (millis() - lastMeasurementTime) * pulses);
    input = alpha * measure + (1.0 - alpha) * input;
    pulses = 0;
    lastMeasurementTime= millis();
}


void StirringController::incrementPulses()
{

    if(micros() - debouncePreviousTime > DEBOUNCE_TIME){
        ++pulses;
       debouncePreviousTime = micros();
    }  
}



void StirringController::adjustOutputSignalManually(uint8_t _pwmValue)
{
    output = _pwmValue;
    analogWrite(motorAPin, output);
}


StirringController::~StirringController()
{
    digitalWrite(motorAPin, LOW);  
    pinMode(motorAPin, INPUT);      
    ledcDetachPin(motorAPin);
    pinMode(encoderPhaseAPin, INPUT);       
    detachInterrupt(digitalPinToInterrupt(encoderPhaseAPin));
}


