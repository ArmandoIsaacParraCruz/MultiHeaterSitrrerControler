#pragma once
#include "Controller.h"
#include <max6675.h>

class HeatingController: public Controller{
    public:
        HeatingController(  uint8_t _heatingResistorPin, 
                            double _kp,
                            double _ki,
                            double _kd,
                            uint8_t _clkPin,
                            uint8_t _csPin, 
                            uint8_t _soPin);
     
        static volatile uint32_t lastZeroCrossingTime;
        static volatile uint32_t semicyclesCounter;
        static const uint32_t DEBOUNCE_TIME = 8300;
        static uint32_t maxNumberOfSemicycles;
        static uint8_t zeroCrossingPin;
        
        void adjustOutputSignal() override;
        void updateInput() override;
        void configureHeatingControllerPins(uint8_t &_zeroCrossingPin, void (*_zeroCrossingInterruptFunction)());
        ~HeatingController();
        

    private:
        MAX6675 max6675Sensor;
        uint8_t heatingResistorPin;    
};



