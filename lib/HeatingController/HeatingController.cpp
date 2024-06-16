#include "HeatingController.h"

volatile uint8_t HeatingController::semicyclesCounter = 0; 

HeatingController::HeatingController(double _kp, double _ki, double _kd, double _alpha, double _beta, double _tau1, double _tau2)
    : Controller(_kp, _ki, _kd), alpha(_alpha), beta(_beta), tau1(_tau1), tau2(_tau2)
{
}

void HeatingController::begin(uint8_t _heatingResistorPin, uint8_t _cs)
{
    heatingResistorPin = _heatingResistorPin;
    cs = _cs;
    pinMode(heatingResistorPin, OUTPUT);
    digitalWrite(heatingResistorPin, LOW);
}


void HeatingController::adjustOutputSignal()
{
    if(semicyclesCounter < output) {
        digitalWrite(heatingResistorPin, HIGH);
    } else {
        digitalWrite(heatingResistorPin, LOW);
    }
    
}

void HeatingController::adjustOutputSignalManually(uint8_t semicycles){

    output = semicycles;
}

void HeatingController::updateInput()
{
    uint16_t v;
    // Leer datos
    MCP23017::digitalWrite(cs, LOW);
    byte msb = SPI.transfer(0x00);
    byte lsb = SPI.transfer(0x00);
    MCP23017::digitalWrite(cs, HIGH);
    // Convertir bits a temperatura
    int temperature = ((msb << 8) | lsb) >> 3;

    input = temperature * 0.25;

    computeOutput();
    
    double error = setpoint - input;
    output = applyLeadLagCompensation(output, error);
}

double HeatingController::applyLeadLagCompensation(double pidOutput, double error)
{
    // Calcular salida del compensador de adelanto
    double leadCompensatorOutput = (tau1 * pidOutput + error) / (alpha * tau1 + 1);

    // Calcular salida del compensador de atraso
    double lagCompensatorOutput = (beta * tau2 * leadCompensatorOutput + leadCompensatorOutput) / (tau2 + 1);

    return constrain(lagCompensatorOutput, 0, 120); // Ajustar los límites de la salida según sea necesario
}


uint8_t HeatingController::getSemicyclesCounter() 
{
    return semicyclesCounter;
}

void HeatingController::setSemicyclesCounter(uint8_t value)
{
    semicyclesCounter = value;
}

void HeatingController::incrementSemicyclesCounter()
{
    ++semicyclesCounter;
}





