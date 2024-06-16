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
        float getTemperature();
        void updateInput() override;
        static uint8_t getSemicyclesCounter();
        static void setSemicyclesCounter(uint8_t value);
        static void incrementSemicyclesCounter();
        static volatile uint8_t semicyclesCounter;
        

    private:
        uint8_t heatingResistorPin;
        uint8_t cs;
        double alpha, beta, tau1, tau2;
        double applyLeadLagCompensation(double pidOutput, double error);
};



