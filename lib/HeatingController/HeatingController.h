#pragma once
#include <Wire.h>
#include <SPI.h>
#include "Controller.h"
#include "MCP23017.h"

class HeatingController: public Controller{
    public:
        HeatingController(double _kp, double _ki, double _kd, double _alpha, double _beta, double _tau1, double _tau2);
        void begin(uint8_t _heatingResistorPin, uint8_t cs);
        void adjustOutputSignal() override;
        void adjustOutputSignalManually(uint8_t semicycles);
        void updateInput() override;
        uint8_t getSemicyclesCounter();
        void setSemicyclesCounter(uint8_t value);
        void incrementSemicyclesCounter();
        

    private:
        uint8_t heatingResistorPin;
        uint8_t cs;
        static volatile uint8_t semicyclesCounter;
        double alpha, beta, tau1, tau2;
         double applyLeadLagCompensation(double pidOutput, double error);
};



