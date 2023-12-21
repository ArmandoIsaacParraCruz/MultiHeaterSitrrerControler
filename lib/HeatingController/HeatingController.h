#pragma once
#include "Controller.h"
#include "SPI.h"

class HeatingController: public Controller{
    public:
        HeatingController(  double _kp,
                            double _ki,
                            double _kd,
                            uint8_t _csPin);
        void adjustOutputSignal() override;
        void updateInput() override;
        float getTemperature();

    private:
        static const double MAX_NUMBER_OF_SEMICYCLES;
        static const double MIN_NUMBER_OF_SEMICYCLES;
        uint8_t heatingResistorPin;
        uint8_t csPin; 
        
};



